#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {
/**
 * @return JointData containing positions, velocities, and efforts
 *
 * @brief Retrieves the current robot joint positions, velocities, and efforts.
 *
 * @details
 * Uses a single mutex lock to safely access the most recently cached joint data.
 * The data is updated periodically by the polling thread.
 *
 * Returns empty JointData if:
 * - Mutex cannot be acquired (non-blocking)
 * - Polling hasn't started yet (cache is empty)
 *
 * Returns zero velocities/efforts if not available or size mismatch.
 */
JointData HdrDriver::GetJointData() const {
  std::unique_lock<std::mutex> lock(position_mtx_, std::try_to_lock);

  JointData data;

  if (!lock.owns_lock()) {
    return data;
  }

  data.positions = cached_position_;

  if (!cached_velocity_.empty() && cached_velocity_.size() == cached_position_.size()) {
    data.velocities = cached_velocity_;
  } else {
    data.velocities = std::vector<double>(cached_position_.size(), 0.0);
  }

  if (!cached_effort_.empty() && cached_effort_.size() == cached_position_.size()) {
    data.efforts = cached_effort_;
  } else {
    data.efforts = std::vector<double>(cached_position_.size(), 0.0);
  }

  return data;
}

/**
 * @param stream_ver Stream version string (e.g., "1.0.0")
 * @param timeout_ms Timeout in milliseconds for handshake response (default: 5000ms)
 * @return True on success, false on failure
 *
 * @brief Performs socket Stream handshake with robot controller and waits for confirmation.
 *
 * @details
 * Sends HANDSHAKE command with major, minor, and patch version numbers parsed from stream_ver
 * and waits for handshake_ack response: {"type":"handshake_ack","ok":true}
 *
 * Must be called before StartPolling(). Blocks until handshake is confirmed or timeout occurs.
 *
 * The version string format is "major.minor.patch" (e.g., "1.0.0")
 * - major: First number (e.g., 1 from "1.0.0")
 * - minor: Second number (e.g., 0 from "1.0.0")
 * - patch: Third number (e.g., 0 from "1.0.0")
 */
bool HdrDriver::DoHandshake(const std::string& stream_ver, int timeout_ms) {
  if (!stream_socket_) {
    std::cerr << "[ERROR] socket Stream not initialized" << std::endl;
    return false;
  }

  // Parse version string "major.minor.patch"
  int major = 1;  // Default values
  int minor = 0;
  int patch = 0;

  try {
    std::istringstream version_stream(stream_ver);
    std::string major_str, minor_str, patch_str;

    // Parse "major.minor.patch"
    if (std::getline(version_stream, major_str, '.')) {
      major = std::stoi(major_str);
      if (std::getline(version_stream, minor_str, '.')) {
        minor = std::stoi(minor_str);
        if (std::getline(version_stream, patch_str, '.')) {
          patch = std::stoi(patch_str);
        }
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "[WARN] Failed to parse stream version '" << stream_ver
              << "', using defaults (1.0.0): " << e.what() << std::endl;
    major = 1;
    minor = 0;
    patch = 0;
  }

  std::cout << "[INFO] Performing handshake - Version: " << major << "." << minor << "." << patch
            << std::endl;

  try {
    nlohmann::json hs = {{"cmd", "HANDSHAKE"},
                         {"payload", {{"major", major}, {"minor", minor}, {"patch", patch}}}};

    stream_socket_->SendData(hs.dump() + "\n");

    auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(timeout_ms);

    while (true) {
      // Check timeout
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > timeout) {
        std::cerr << "[ERROR] Handshake timeout after " << timeout_ms << "ms" << std::endl;
        return false;
      }

      std::string line = stream_socket_->ReceiveData();

      if (line.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      try {
        nlohmann::json response = nlohmann::json::parse(line);

        // Check for error response
        if (response.contains("error")) {
          std::cerr << "[ERROR] Handshake failed - Server returned error:" << std::endl;
          std::cerr << "  Error: " << response["error"].get<std::string>() << std::endl;
          if (response.contains("message")) {
            std::cerr << "  Message: " << response["message"].get<std::string>() << std::endl;
          }
          if (response.contains("hint")) {
            std::cerr << "  Hint: " << response["hint"].get<std::string>() << std::endl;
          }
          return false;
        }

        if (response.contains("type") && response["type"] == "handshake_ack") {
          if (response.contains("ok") && response["ok"].get<bool>() == true) {
            std::cout << "[INFO] Handshake successful - ACK received" << std::endl;

            if (response.contains("major") && response.contains("minor") &&
                response.contains("patch")) {
              int server_major = response["major"].get<int>();
              int server_minor = response["minor"].get<int>();
              int server_patch = response["patch"].get<int>();

              std::cout << "[INFO] Server version: " << server_major << "." << server_minor << "."
                        << server_patch << std::endl;
              std::cout << "[INFO] Client version: " << major << "." << minor << "." << patch
                        << std::endl;

              if (server_major != major) {
                std::cerr << "[ERROR] Version incompatibility detected!" << std::endl;
                std::cerr << "[ERROR] Server major version (" << server_major
                          << ") != Client major version (" << major << ")" << std::endl;
                std::cerr << "[ERROR] Major version must match for compatibility" << std::endl;
                return false;
              }

              if (server_minor != minor || server_patch != patch) {
                std::cout << "[INFO] Minor/patch version difference detected (compatible)"
                          << std::endl;
              }
            }

            return true;
          } else {
            std::cerr << "[ERROR] Handshake failed - ok=false: " << response.dump() << std::endl;
            return false;
          }
        }

      } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "[WARN] Failed to parse handshake response: " << e.what() << std::endl;
        std::cerr << "Raw line: " << line << std::endl;
        continue;
      }
    }

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] Handshake exception: " << e.what() << std::endl;
    return false;
  }
}

/**
 * @param hz Polling frequency in Hertz (Hz)
 * @return True if polling started successfully, false otherwise
 *
 * @brief Starts a background thread that periodically queries the robot's joint data.
 *
 * @details
 * Launches a polling thread that retrieves joint position and velocity data at the
 * specified frequency via the MONITOR command using joint_states endpoint.
 *
 * The retrieved data is stored in cached_position_ and cached_velocity_ and protected
 * by a mutex. joint_states provides:
 * - position: Joint positions in degrees (converted to radians)
 * - velocity: Joint velocities in degrees/s (converted to radians/s)
 * - effort: Joint efforts/torques
 *
 * All angles are converted from degrees to radians, and speeds from degrees/s to radians/s.
 * If polling is already running, this function returns true immediately.
 *
 * Must be called after DoHandshake().
 */
bool HdrDriver::StartPolling(int hz) {
  if (polling_)
    return true;
  if (hz <= 0)
    return false;
  if (!stream_socket_)
    return false;

  polling_ = true;

  polling_thread_ = std::thread([this, hz]() {
    pthread_setname_np(pthread_self(), "hdr_polling");

    struct sched_param param;
    param.sched_priority = 80;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    const auto interval = std::chrono::milliseconds(1000 / hz);

    nlohmann::json mon = {{"cmd", "MONITOR"},
                          {"payload",
                           {{"id", 1},
                            {"method", "GET"},
                            {"period_ms", interval.count()},
                            {"url", "/project/robot/joints/joint_states"},
                            {"args", nlohmann::json::object()}}}};
    stream_socket_->SendData(mon.dump() + "\n");

    int consecutive_failures = 0;
    const int max_failures = 5;

    auto next_read_time = std::chrono::steady_clock::now();

    while (polling_) {
      std::this_thread::sleep_until(next_read_time);
      next_read_time += interval;

      std::string latest_line;
      int messages_read = 0;

      std::string line = stream_socket_->ReceiveData();
      if (line.empty()) {
        consecutive_failures++;
        if (consecutive_failures >= max_failures) {
          std::cerr << "[ERROR] Too many consecutive failures, stopping polling" << std::endl;
          polling_ = false;
          break;
        }
        continue;
      }
      latest_line = std::move(line);
      messages_read++;

      while (true) {
        line = stream_socket_->TryReceiveLineNonBlocking();
        if (line.empty()) {
          break;
        }
        latest_line = std::move(line);
        messages_read++;
      }

      try {
        nlohmann::json j = nlohmann::json::parse(latest_line);

        if (j.contains("type") && j["type"] == "data" && j.contains("result")) {
          auto result = j["result"];

          std::vector<double> positions;
          std::vector<double> velocities;
          std::vector<double> efforts;

          if (result.contains("position") && result["position"].is_array()) {
            auto pos_array = result["position"];
            positions.reserve(pos_array.size());

            // Convert degrees to radians
            for (const auto& pos : pos_array) {
              if (pos.is_number()) {
                positions.push_back(pos.get<double>() * util::kDegToRad);
              }
            }
          }

          if (result.contains("velocity") && result["velocity"].is_array()) {
            auto vel_array = result["velocity"];
            velocities.reserve(vel_array.size());

            for (const auto& vel : vel_array) {
              if (vel.is_number()) {
                velocities.push_back(vel.get<double>() * util::kDegToRad);
              }
            }
          }

          if (result.contains("effort") && result["effort"].is_array()) {
            auto eff_array = result["effort"];
            efforts.reserve(eff_array.size());

            for (const auto& eff : eff_array) {
              if (eff.is_number()) {
                efforts.push_back(eff.get<double>());
              }
            }
          }

          if (!positions.empty()) {
            std::lock_guard<std::mutex> lock(position_mtx_);
            cached_position_ = std::move(positions);

            if (!velocities.empty() && velocities.size() == cached_position_.size()) {
              cached_velocity_ = std::move(velocities);
            } else {
              cached_velocity_ = std::vector<double>(cached_position_.size(), 0.0);
            }

            if (!efforts.empty() && efforts.size() == cached_position_.size()) {
              cached_effort_ = std::move(efforts);
            } else {
              cached_effort_ = std::vector<double>(cached_position_.size(), 0.0);
            }

            consecutive_failures = 0;
          }
        }

      } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "[ERROR] JSON parse error: " << e.what() << std::endl;
        std::cerr << "Failed line: " << latest_line << std::endl;
        consecutive_failures++;
      } catch (const std::exception& e) {
        std::cerr << "[ERROR] Error processing data: " << e.what() << std::endl;
        consecutive_failures++;
      }

      // Check consecutive failures
      if (consecutive_failures >= max_failures) {
        std::cerr << "[ERROR] Stopping polling due to repeated errors" << std::endl;
        polling_ = false;
        break;
      }
    }
  });

  return true;
}

/**
 * @brief Stops the background polling thread if it is running.
 *
 * @details
 * Sends STOP command to halt monitoring, signals the polling thread to stop,
 * and joins the thread if it is joinable. Ensures proper cleanup and prevents
 * dangling threads. Also stops the send thread.
 */
void HdrDriver::StopPolling() {
  if (!polling_)
    return;

  if (stream_socket_) {
    try {
      nlohmann::json stop_cmd = {{"cmd", "STOP"}, {"payload", {{"id", 1}}}};
      stream_socket_->SendData(stop_cmd.dump() + "\n");
    } catch (...) {
      // Ignore errors during shutdown
    }
  }

  polling_ = false;
  pacer_running_ = false;

  if (polling_thread_.joinable())
    polling_thread_.join();
  if (pacer_thread_.joinable())
    pacer_thread_.join();

  StopSending();
}

/**
 * @param position Joint angles in radians
 * @param hz Command frequency in Hz
 * @param look_ahead_time Look-ahead time in seconds
 * @return True on success, false on failure
 *
 * @brief Sets robot joint position via async trajectory insertion.
 *
 * @details
 * Queues a CONTROL command to insert a trajectory point into the robot's motion buffer.
 * The command is sent asynchronously via the send thread, preventing blocking.
 *
 * Parameters:
 * - position: Target joint angles (converted from radians to degrees)
 * - hz: Control frequency to calculate interval (interval = 1/hz)
 * - look_ahead_time: Delay time for trajectory execution
 *
 * The async queue prevents network delays from blocking the control loop at 500Hz.
 * If queue is full, oldest commands are dropped with a warning.
 */
bool HdrDriver::SetRobotPosition(const std::vector<double>& position, int hz,
                                 double look_ahead_time) const {
  if (!send_running_) {
    std::cerr << "[ERROR] Send thread not running. Call StartSending() first." << std::endl;
    return false;
  }

  try {
    std::vector<double> degrees;
    degrees.reserve(position.size());
    for (const auto& val : position) {
      degrees.push_back(val * util::kRadToDeg);
    }

    double interval = 1.0 / static_cast<double>(hz);

    nlohmann::json cmd = {
        {"cmd", "CONTROL"},
        {"payload",
         {{"method", "POST"},
          {"url", "/project/robot/trajectory/joint_traject_insert_point"},
          {"args", nlohmann::json::object()},
          {"body",
           {{"interval", interval}, {"look_ahead_time", look_ahead_time}, {"point", degrees}}}}}};

    std::string msg = cmd.dump() + "\n";

    {
      std::unique_lock<std::mutex> lock(send_mtx_, std::try_to_lock);

      if (!lock.owns_lock()) {
        return true;
      }

      if (send_queue_.size() >= kMaxQueueSize) {
        std::cerr << "[WARN] Send queue full (" << send_queue_.size()
                  << " commands), command may be delayed or dropped" << std::endl;
        send_queue_.pop();
      }

      send_queue_.push(std::move(msg));
    }

    send_cv_.notify_one();

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] SetRobotPosition failed: " << e.what() << std::endl;
    return false;
  }
}
/**
 * @return A vector of script commands for robot control
 *
 * @brief Generates script commands for online trajectory control.
 *
 * @details
 * Generates a sequence of robot language commands for real-time trajectory control:
 * - rl.stop: Stop any running robot language programs
 * - rl.reinit: Reinitialize the robot language interpreter
 * - rl.i wait di1: Wait for digital input 1 signal
 * - rl.i end: End of inline commands
 * - rl.start: Start program execution
 */
std::vector<std::string> GenerateScriptCommands() {
  std::vector<std::string> script = {"rl.stop", "rl.reinit", "rl.i wait di1", "rl.i end",
                                     "rl.start"};
  return script;
}

/**
 * @param hz Sampling frequency in Hz (1-500)
 * @return True on success, false on failure
 *
 * @brief Setup robot job program for online trajectory control.
 *
 * @details
 * Configures the robot for online trajectory control by:
 * - Stopping and reinitializing existing programs
 * - Setting up the control timing parameters
 * - Starting the robot language program
 *
 * Must be called after DoHandshake() and before sending trajectory commands.
 * The frequency parameter determines the control interval (interval = 1/hz).
 */
bool HdrDriver::SetJobProgram(int hz) const {
  if (hz <= 0 || hz > 500) {
    std::cerr << "[ERROR] Invalid frequency: " << hz << " Hz (valid range: 1-500)" << std::endl;
    return false;
  }

  try {
    const double interval = 1.0 / static_cast<double>(hz);

    auto script_commands = GenerateScriptCommands();

    if (script_commands.empty()) {
      std::cerr << "[ERROR] Failed to generate script commands" << std::endl;
      return false;
    }

    auto [result, success] = ExecuteCommand(script_commands, interval);
    return success;

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] SetJobProgram failed: " << e.what() << std::endl;
    return false;
  } catch (...) {
    std::cerr << "[ERROR] SetJobProgram failed: unknown error" << std::endl;
    return false;
  }
}

/**
 * @return std::pair<nlohmann::json, bool> Command execution result and success status
 *
 * @brief Clear external stop state by setting relay value.
 *
 * @details
 * Sets the relay "fb0.di23" to value 1 to clear external stop conditions.
 * This is required to enable remote control mode and allow trajectory commands.
 *
 * The function validates the relay operation and provides detailed error information
 * if the operation fails.
 */
std::pair<nlohmann::json, bool> HdrDriver::RelayExternalStopClear() const {
  auto [relay_result, relay_success] = SetRelayValue("fb0.di23", 1);

  if (!relay_success) {
    std::cerr << "[ERROR] Failed to clear external stop via relay fb0.di23: " << relay_result.dump()
              << std::endl;
    return {relay_result, false};
  }

  std::cout << "[INFO] Successfully cleared external stop via relay fb0.di23" << std::endl;

  return {relay_result, true};
}

/**
 * @return True if send thread started successfully, false otherwise
 *
 * @brief Starts a background thread that sends queued commands asynchronously.
 *
 * @details
 * Launches a send thread that processes commands from the queue and sends them
 * via socket Stream. This prevents blocking the control loop when sending commands.
 *
 * Commands are sent in FIFO order. If the queue is full, old commands are discarded
 * to prevent memory overflow.
 *
 * Must be called after DoHandshake().
 */
bool HdrDriver::StartSending() {
  if (send_running_)
    return true;
  if (!stream_socket_) {
    std::cerr << "[ERROR] socket Stream not initialized" << std::endl;
    return false;
  }

  send_running_ = true;

  send_thread_ = std::thread([this]() {
    std::cout << "[INFO] Send thread started" << std::endl;

    while (send_running_) {
      std::unique_lock<std::mutex> lock(send_mtx_);

      send_cv_.wait(lock, [this] { return !send_queue_.empty() || !send_running_; });

      if (!send_running_)
        break;

      while (!send_queue_.empty()) {
        std::string cmd = std::move(send_queue_.front());
        send_queue_.pop();

        lock.unlock();

        try {
          stream_socket_->SendData(cmd);
        } catch (const std::exception& e) {
          std::cerr << "[ERROR] Send failed: " << e.what() << std::endl;
        }

        lock.lock();
      }
    }

    std::cout << "[INFO] Send thread stopped" << std::endl;
  });

  return true;
}

/**
 * @brief Stops the background send thread if it is running.
 *
 * @details
 * Signals the send thread to stop, notifies via condition variable,
 * and joins the thread if it is joinable. Ensures proper cleanup and prevents
 * dangling threads.
 */
void HdrDriver::StopSending() {
  if (!send_running_)
    return;

  send_running_ = false;
  send_cv_.notify_all();

  if (send_thread_.joinable())
    send_thread_.join();

  std::lock_guard<std::mutex> lock(send_mtx_);
  std::queue<std::string> empty;
  std::swap(send_queue_, empty);
}

}  // namespace hdrcl
