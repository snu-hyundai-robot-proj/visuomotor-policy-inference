#ifndef HDR_DRIVER_H_
#define HDR_DRIVER_H_

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <queue>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <unordered_set>

#include "hdr_client_driver/request_lib.h"
#include "hdr_client_driver/socket_manager.h"
#include "hdr_client_driver/util.h"

namespace hdrcl {

/**
 * @brief Joint measurement data structure
 *
 * @details
 * Contains joint positions, velocities, and efforts measured from the robot.
 * All values are in radians, radians/second, and torque units respectively.
 */
struct JointData {
  std::vector<double> positions;   ///< Joint positions in radians
  std::vector<double> velocities;  ///< Joint velocities in radians/s
  std::vector<double> efforts;     ///< Joint efforts/torques

  /**
   * @return True if data contains valid position information
   */
  bool IsValid() const { return !positions.empty(); }

  /**
   * @return Number of joints
   */
  size_t Size() const { return positions.size(); }
};

/**
 * @brief Main client class for interacting with HDR OpenAPI interface.
 *
 * @details
 * This class provides a comprehensive interface to HD Hyundai Robotics controllers
 * via HTTP/socket Stream protocols. It manages robot state, position control, I/O operations,
 * and system configuration through RESTful API calls and socket Stream communication.
 *
 * Key features:
 * - Position polling and control via socket Stream
 * - Robot state management and monitoring
 * - Emergency stop and safety operations
 * - File system and project management
 * - PLC I/O and relay control
 * - Console command execution
 * - Automatic remote mode validation
 *
 * @class HdrDriver
 */
class HdrDriver {
 public:
  HdrDriver(const std::string& robot_ip);
  ~HdrDriver();

  // ────────────────────────────────────────────────────────────────────────────
  // socket Stream handshake and polling control
  // ────────────────────────────────────────────────────────────────────────────
  bool DoHandshake(const std::string& stream_ver, int timeout_ms = 5000);
  bool StartPolling(int hz);
  void StopPolling();
  bool StartSending();
  void StopSending();

  // ────────────────────────────────────────────────────────────────────────────
  // Joint data access for hardware interface
  // ────────────────────────────────────────────────────────────────────────────

  JointData GetJointData() const;
  bool SetRobotPosition(const std::vector<double>& position, int hz, double look_ahead_time) const;
  bool SetJobProgram(int hz) const;
  std::pair<nlohmann::json, bool> RelayExternalStopClear() const;

  // ────────────────────────────────────────────────────────────────────────────
  // Version and system information
  // ────────────────────────────────────────────────────────────────────────────

  double GetApiVersion() const;
  double GetSysVersion() const;

  // ────────────────────────────────────────────────────────────────────────────
  // Project control and monitoring
  // ────────────────────────────────────────────────────────────────────────────

  std::pair<nlohmann::json, bool> GetProjectRgen() const;
  std::pair<nlohmann::json, bool> GetProjectJobsInfo() const;
  std::pair<nlohmann::json, bool> GetEmergencyStop() const;
  std::pair<nlohmann::json, bool> PostProjectReloadUpdateJobs() const;
  std::pair<nlohmann::json, bool> PostProjectDeleteJob(const std::string& path) const;

  // ────────────────────────────────────────────────────────────────────────────
  // Controller configuration and I/O management
  // ────────────────────────────────────────────────────────────────────────────

  std::pair<nlohmann::json, bool> GetControlOpCnd() const;
  std::pair<nlohmann::json, bool> GetControlIosDio(const std::string& raw_type, int blk_no,
                                                   int sig_no) const;
  std::pair<nlohmann::json, bool> GetControlIosSio(const std::string& raw_type, int sig_no) const;
  std::pair<nlohmann::json, bool> GetControlUcsNos() const;
  std::pair<nlohmann::json, bool> PostControlIosDio(const std::string& raw_type, int blk_no,
                                                    int sig_no, int val) const;
  std::pair<nlohmann::json, bool> PutControlOpCnd(bool playback_mode, double step_goback_max_spd,
                                                  int ucrd_num) const;

  // ────────────────────────────────────────────────────────────────────────────
  // Robot information and command interface
  // ────────────────────────────────────────────────────────────────────────────

  std::pair<nlohmann::json, bool> GetRobotMotorState() const;
  std::pair<nlohmann::json, bool> GetRobotCurTool() const;
  std::pair<nlohmann::json, bool> GetRobotTools() const;
  std::pair<nlohmann::json, bool> GetRobotToolsT(int tool_number) const;
  std::pair<nlohmann::json, bool> PostRobotToolNo(int tool_no) const;
  std::pair<nlohmann::json, bool> PostRobotCrdSys(int crd_sys) const;
  std::pair<nlohmann::json, bool> PostRobotMotorPower() const;
  std::pair<nlohmann::json, bool> PostRobotOperation(bool start) const;
  std::pair<nlohmann::json, bool> GetRobotPoCur(int task_no, int crd, int ucrd_no,
                                                bool mechinfo) const;
  std::pair<nlohmann::json, bool> PostRobotEmergencyStop() const;
  std::pair<nlohmann::json, bool> PostRobotEmergencyStopTest(int step_no, int stop_at,
                                                             bool stop_at_corner,
                                                             int category) const;
  std::pair<nlohmann::json, bool> GetJointTrajBuffAvail() const;
  std::pair<nlohmann::json, bool> PostInitJointTrajectory() const;
  std::pair<nlohmann::json, bool> PostInsertJointTrajectoryPoints(
      const nlohmann::json& trajectories) const;

  // ────────────────────────────────────────────────────────────────────────────
  // PLC I/O and relay access
  // ────────────────────────────────────────────────────────────────────────────

  std::pair<nlohmann::json, bool> GetRelayValue(const std::string& name, int st, int len) const;
  std::pair<nlohmann::json, bool> SetRelayValue(const std::string& name, double value) const;

  // ────────────────────────────────────────────────────────────────────────────
  // System date/time and log management
  // ────────────────────────────────────────────────────────────────────────────

  std::pair<nlohmann::json, bool> GetDateTime() const;
  std::pair<nlohmann::json, bool> PutDateTime(int year, int mon, int day, int hour, int min,
                                              int sec) const;
  std::pair<nlohmann::json, bool> GetLogManager(uint64_t n_item, const std::string& cat_p,
                                                std::optional<uint64_t> id_min,
                                                std::optional<uint64_t> id_max,
                                                std::optional<std::string> ts_min,
                                                std::optional<std::string> ts_max) const;

  // ────────────────────────────────────────────────────────────────────────────
  // File system management
  // ────────────────────────────────────────────────────────────────────────────

  std::pair<nlohmann::json, bool> GetFiles(const std::string& path) const;
  std::pair<nlohmann::json, bool> GetFileInfo(const std::string& path) const;
  std::pair<nlohmann::json, bool> GetFileExist(const std::string& path) const;
  std::pair<nlohmann::json, bool> PostMkdir(const std::string& path) const;
  std::pair<nlohmann::json, bool> PostDeleteFile(const std::string& path) const;
  std::pair<nlohmann::json, bool> GetFileList(const std::string& path, bool incl_file,
                                              bool incl_dir) const;
  std::pair<nlohmann::json, bool> PostRenameFile(const std::string& pathname_from,
                                                 const std::string& pathname_to) const;
  std::pair<nlohmann::json, bool> PostFiles(const std::string& target_file,
                                            const std::string& source_file) const;

  // ────────────────────────────────────────────────────────────────────────────
  // Task control APIs
  // ────────────────────────────────────────────────────────────────────────────

  std::pair<nlohmann::json, bool> PostCurProgCnt(int pno, int sno, int fno, int ext_sel) const;
  std::pair<nlohmann::json, bool> PostSetCurPcIdx(int idx) const;
  std::pair<nlohmann::json, bool> PostReleaseWait() const;
  std::pair<nlohmann::json, bool> PostReset() const;
  std::pair<nlohmann::json, bool> PostAssignVar(const std::string& name, const std::string& scope,
                                                const std::string& expr, bool save) const;
  std::pair<nlohmann::json, bool> PostSolveExpr(const std::string& scope,
                                                const std::string& expr) const;
  std::pair<nlohmann::json, bool> PostExecuteMove(const std::string& stmt, int task_no) const;

  // ────────────────────────────────────────────────────────────────────────────
  // Console command interface
  // ────────────────────────────────────────────────────────────────────────────

  std::pair<nlohmann::json, bool> ExecuteCommand(const std::vector<std::string>& commands,
                                                 int period_ms) const;

 private:
  // ────────────────────────────────────────────────────────────────────────────
  // Core API call wrapper with error handling
  // ────────────────────────────────────────────────────────────────────────────

  /**
   * @brief Generic API call wrapper with error handling.
   *
   * @tparam EnableLogging Enable/disable API response logging
   * @tparam Callable Function type for API call
   * @tparam Args Argument types for the API call
   *
   * @param endpoint API endpoint path for logging
   * @param fn Function to execute (GET, POST, PUT, DELETE)
   * @param args Arguments to pass to the function
   *
   * @return std::pair<nlohmann::json, bool> Response JSON and success status
   *
   * @details
   * This template function provides:
   * - Comprehensive error handling with JSON parsing protection
   * - Optional API response logging for debugging
   * - Consistent error response format
   */
  template <bool EnableLogging = true, typename Callable, typename... Args>
  std::pair<nlohmann::json, bool> CallApi(const std::string& endpoint, Callable&& fn,
                                          Args&&... args) const {
    // Helper function to check success
    constexpr auto IsSuccess = [](int code) noexcept { return code / 100 == 2; };

    // Helper function to create error response
    const auto MakeError = [](std::string_view what) { return nlohmann::json{{"error", what}}; };

    try {
      auto [body, status] =
          std::invoke(std::forward<Callable>(fn), endpoint, std::forward<Args>(args)...);

      if constexpr (EnableLogging) {
        util::LogApiResponse(endpoint, body, status);
      }

      return {std::move(body), IsSuccess(status)};
    } catch (const nlohmann::json::exception& e) {
      return {MakeError("JSON parsing error: " + std::string(e.what())), false};
    } catch (const std::exception& e) {
      return {MakeError("API call failed: " + std::string(e.what())), false};
    } catch (...) {
      return {MakeError("Unknown error occurred"), false};
    }
  }

  // ────────────────────────────────────────────────────────────────────────────
  // Joint data polling and caching
  // ────────────────────────────────────────────────────────────────────────────

  mutable std::mutex position_mtx_;      ///< Mutex for thread-safe access to cached data
  std::vector<double> cached_position_;  ///< Latest robot joint positions [rad] (EMA smoothed)
  std::vector<double> cached_velocity_;  ///< Latest robot joint velocities [rad/s]
  std::vector<double> cached_effort_;    ///< Latest robot joint efforts/torques
  static constexpr double kSmoothingAlpha = 0.7;  ///< EMA smoothing factor (0.7 = 70% new, 30% old)
  std::atomic<bool> polling_ = false;             ///< Flag indicating whether polling is active
  std::thread polling_thread_;  ///< Background thread for polling robot joint data

  // ────────────────────────────────────────────────────────────────────────────
  // Async command sending queue and thread
  // ────────────────────────────────────────────────────────────────────────────

  mutable std::mutex send_mtx_;                 ///< Mutex for thread-safe access to send queue
  mutable std::condition_variable send_cv_;     ///< Condition variable for queue notification
  mutable std::queue<std::string> send_queue_;  ///< Command queue for async sending
  mutable std::atomic<bool> send_running_ =
      false;                                   ///< Flag indicating whether send thread is active
  mutable std::thread send_thread_;            ///< Background thread for async command sending
  static constexpr size_t kMaxQueueSize = 50;  ///< Maximum queue size to prevent overflow

  // ────────────────────────────────────────────────────────────────────────────
  // Network configuration
  // ────────────────────────────────────────────────────────────────────────────

  std::string robot_ip_{"192.168.1.150"};            ///< Robot IP address
  int openapi_port_{8888};                           ///< OpenAPI HTTP port
  std::shared_ptr<request::RequestLib> api_client_;  ///< HTTP client wrapper

  std::string job_mode_ = "UDP";  ///< Communication mode (UDP/TCP)
  int job_network_port_{8000};    ///< UDP/TCP port for trajectory streaming
  std::shared_ptr<network::SocketManager>
      job_socket_;  ///< Socket manager for UDP/TCP communication

  std::string stream_mode_ = "TCP_CLIENT";                 ///< socket Stream mode
  int stream_port_{49000};                                ///< socket Stream port
  std::shared_ptr<network::SocketManager> stream_socket_;  ///< socket Stream for MONITOR/CONTROL

  std::thread pacer_thread_;                ///< Pacer thread (if needed)
  std::atomic<bool> pacer_running_{false};  ///< Pacer running flag
};

}  // namespace hdrcl

#endif  // HDR_DRIVER_H_
