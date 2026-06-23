#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {
/**
 * @return A pair:
 * - JSON object (e.g., `{ "value": true }`)
 * - `true` if request succeeded (HTTP 2xx), otherwise `false`
 *
 * @brief Get the robot motor state (ON/OFF).
 *
 * @details
 * Queries whether the robot servo motor is currently powered on.
 * Useful for monitoring the robot's readiness for motion commands.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/1-motor_on_state
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetRobotMotorState() const {
  return CallApi<false>("/project/robot/motor_on_state",
                        [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}

/**
 * @param task_no Task index (0–7)
 * @param crd Coordinate system (-1 to 3)
 * @param ucrd_no User coordinate system number
 * @param mechinfo Whether to include mechanical info in response
 *
 * @return JSON pose information and success flag
 *
 * @brief Get the current robot pose (position & orientation).
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/2-po_cur
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetRobotPoCur(int task_no, int crd, int ucrd_no,
                                                         bool mechinfo) const {
  if (task_no < util::kTaskNoMin || task_no > util::kTaskNoMax)
    return {nlohmann::json{{"error", "Invalid task_no: out of range."}}, false};
  if (crd < util::kCrdNoMin || crd > util::kCrdNoMax)
    return {nlohmann::json{{"error", "Invalid crd: out of range."}}, false};

  nlohmann::json param = {
      {"task_no", task_no}, {"crd", crd}, {"ucrd_no", ucrd_no}, {"mechinfo", mechinfo}};

  return CallApi("/project/robot/po_cur", [this, &param](const std::string& endpoint) {
    return api_client_->Get(endpoint, param.dump());
  });
}

/**
 * @return JSON tool data and success flag
 *
 * @brief Get the currently active tool data.
 *
 * @details
 * Retrieves the currently selected tool information (TCP configuration, weight, etc.).
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/3-cur_tool_data
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetRobotCurTool() const {
  return CallApi("/project/robot/cur_tool_data",
                 [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}

/**
 * @return JSON array of tools and success flag
 *
 * @brief Retrieve all registered tools in the system.
 *
 * @details
 * This includes all tool definitions (e.g., TCP offsets, weights) defined in the robot system.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/4-tools
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetRobotTools() const {
  return CallApi("/project/robot/tools",
                 [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}

/**
 * @param tool_number Tool index (0–31)
 *
 * @return JSON object for the specific tool and success flag
 *
 * @brief Retrieve tool information by tool number.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/5-tools_t
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetRobotToolsT(int tool_number) const {
  return CallApi("/project/robot/tools/t_" + std::to_string(tool_number),
                 [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}

/**
 * @return A pair consisting of:
 * - `nlohmann::json`: Response containing the emergency stop status
 * - `bool`: True if the HTTP status code is 2xx, false otherwise
 *
 * @brief Get the current emergency stop status of the robot.
 *
 * @details
 * Retrieves the current emergency stop state from the robot controller.
 * This sends a GET request to the `/project/robot/emergency_stop` endpoint.
 * The response typically contains information about whether the emergency
 * stop is currently active or inactive.
 *
 * Possible response values may include:
 * - Emergency stop status (active/inactive)
 * - Timestamp of last status change
 * - Additional safety-related information
 *
 * This function is critical for safety monitoring and should be used
 * to check the robot's emergency stop state before performing operations.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/6-emergency_stop
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetEmergencyStop() const {
  return CallApi<false>("/project/robot/emergency_stop",
                        [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}

/**
 * @return JSON result and success flag
 *
 * @brief Turn robot motor ON.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/1-motor-on
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostRobotMotorPower() const {
  auto [current_state_json, state_success] = GetRobotMotorState();
  if (!state_success) {
    nlohmann::json error_response;
    error_response["error"] = "Failed to get current motor state";
    return {error_response, false};
  }

  if (!current_state_json.contains("val") || !current_state_json["val"].is_number_integer()) {
    nlohmann::json error_response;
    error_response["error"] = "Invalid motor state response format";
    return {error_response, false};
  }

  int motor_state = current_state_json["val"];
  if (motor_state == 2) {
    nlohmann::json error_response;
    error_response["error"] = "Motor is busy (state transition in progress)";
    error_response["motor_state"] = motor_state;
    return {error_response, false};
  }

  bool current_power_state = (motor_state == 0);
  if (current_power_state) {
    nlohmann::json response;
    response["message"] = "Motor is already on";
    response["motor_state"] = motor_state;
    response["skipped"] = true;
    return {response, true};
  }

  std::string endpoint = "/project/robot/motor_on";
  auto [command_result, command_success] = CallApi(endpoint, [this](const std::string& ep) {
    return api_client_->Post(ep, "", nlohmann::json::object());
  });

  if (!command_success) {
    return {command_result, false};
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  auto [final_state_json, final_success] = GetRobotMotorState();
  if (!final_success || !final_state_json.contains("val")) {
    nlohmann::json warning_response = command_result;
    warning_response["warning"] = "Command sent but failed to verify final state";
    return {warning_response, true};
  }

  int final_state = final_state_json["val"];
  bool final_power_state = (final_state == 0);

  if (final_power_state) {
    nlohmann::json success_response = command_result;
    success_response["motor_state"] = final_state;
    success_response["verified"] = true;
    return {success_response, true};
  } else {
    nlohmann::json error_response = command_result;
    error_response["error"] = "Motor command failed - state not changed as expected";
    error_response["expected_state"] = 0;
    error_response["actual_state"] = final_state;
    return {error_response, false};
  }
}

/**
 * @param start True to start program execution, false to stop
 *
 * @return JSON response and success flag
 *
 * @brief Start or stop the robot program.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/2-start-stop
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostRobotOperation(bool start) const {
  std::string endpoint = start ? "/project/robot/start" : "/project/robot/stop";
  return CallApi(endpoint, [this](const std::string& ep) {
    return api_client_->Post(ep, "", nlohmann::json::object());
  });
}

/**
 * @param tool_no Tool index to activate (0–31)
 *
 * @return JSON response and success flag
 *
 * @brief Set the current tool number to be used.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/3-tool_no
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostRobotToolNo(int tool_no) const {
  if (tool_no < util::kToolNoMin || tool_no > util::kToolNoMax)
    return {nlohmann::json{{"error", "Invalid tool number: out of range."}}, false};

  nlohmann::json param = {{"val", tool_no}};

  return CallApi("/project/robot/tool_no", [this, &param](const std::string& endpoint) {
    return api_client_->Post(endpoint, "", param);
  });
}

/**
 * @param crd_sys Coordinate system ID (-1: default, 0: base, 1: tool, 2: user1, 3: user2)
 *
 * @return JSON response and success flag
 *
 * @brief Set the robot's coordinate system.
 *
 * @details
 * Sets which coordinate system (base/tool/user-defined) is used for motion and I/O.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/4-crd_sys
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostRobotCrdSys(int crd_sys) const {
  if (crd_sys < util::kCrdNoMin || crd_sys > util::kCrdNoMax)
    return {nlohmann::json{{"error", "Invalid coordinate system number: out of range."}}, false};

  nlohmann::json param = {{"val", crd_sys}};

  return CallApi("/project/robot/crd_sys", [this, &param](const std::string& endpoint) {
    return api_client_->Post(endpoint, "", param);
  });
}

/**
 * @param step_no (Reserved for future use)
 * @param stop_at (Reserved)
 * @param stop_at_corner (Reserved)
 * @param category (Reserved)
 *
 * @return JSON response and success flag
 *
 * @brief Trigger an emergency stop on the robot.
 *
 * @details
 * Immediately halts all robot motion. Intended for emergency safety response.
 * Parameters are currently not sent in this simplified version.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/5-emergency_stop
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostRobotEmergencyStop() const {
  return CallApi("/project/robot/emergency_stop", [this](const std::string& endpoint) {
    return api_client_->Post(endpoint, "", nlohmann::json::object());
  });
}
/**
 * @param step_no Step number for the emergency stop target (1-999)
 * @param stop_at Percentage position where to stop (1-100)
 * @param stop_at_corner Stop type: false = normal stop, true = corner stop
 * @param category Stop category: 0 = immediate stop, 1 = deceleration stop, 2 = pause
 *
 * @return A pair containing the JSON response and a boolean indicating success (true if HTTP 200).
 *
 * @brief Simulate an emergency stop for testing purposes.
 *
 * @details
 * Sends a test emergency stop command to the robot controller with parameter values.
 * This does not perform a real emergency stop, but mimics one to verify system behavior.
 * Parameters:
 * - step_no: Emergency stop target step number, within total step count of current job (1-999)
 * - stop_at: Percentage of specified position where to stop (1-100)
 * - stop_at_corner: Stop type - false: normal stop, true: corner stop
 * - category: Stop category - 0: immediate stop, 1: deceleration stop, 2: pause
 * API Reference:
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/6-emergency_stop_test
 *
 * @throws std::invalid_argument If parameters are out of valid range
 */
std::pair<nlohmann::json, bool> HdrDriver::PostRobotEmergencyStopTest(int step_no, int stop_at,
                                                                      bool stop_at_corner,
                                                                      int category) const {
  // Parameter validation
  if (step_no < 1 || step_no > 999) {
    throw std::invalid_argument("step_no must be between 1 and 999");
  }

  if (stop_at < 1 || stop_at > 100) {
    throw std::invalid_argument("stop_at must be between 1 and 100");
  }

  if (category < 0 || category > 2) {
    throw std::invalid_argument("category must be 0 (immediate), 1 (deceleration), or 2 (pause)");
  }

  try {
    nlohmann::json body = {{"step_no", step_no},
                           {"stop_at", stop_at},
                           {"stop_at_corner", stop_at_corner ? 1 : 0},
                           {"category", category}};

    return CallApi("/project/robot/emergency_stop_test",
                   [this, &body](const std::string& endpoint) {
                     return api_client_->Post(endpoint, "", body);
                   });
  } catch (const nlohmann::json::exception& e) {
    nlohmann::json error_response = {{"error", "JSON processing failed"}, {"message", e.what()}};
    return std::make_pair(error_response, false);
  } catch (const std::exception& e) {
    nlohmann::json error_response = {{"error", "API call failed"}, {"message", e.what()}};
    return std::make_pair(error_response, false);
  }
}
/**
 * @return A pair containing the JSON response and a boolean indicating success (true if HTTP 200).
 *         Response JSON contains "val" field with available buffer count (0-2048).
 *
 * @brief Get the available size of the trajectory buffer.
 *
 * @details
 * Returns the number of available slots in the trajectory buffer that stores joint trajectories.
 * When continuously requesting trajectories, use this function to ensure the buffer has sufficient
 * space before sending new trajectory points. The maximum buffer size is 2048.
 *
 * Response format:
 * - val: Number of currently available buffer slots (max: 2048)
 *
 * Example response:
 * @code{.json}
 * {
 *   "val": 2048
 * }
 * @endcode
 *
 * Supported version: 61.00-00 or higher
 *
 * API Reference:
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/7-joint_traject_buf_avail
 */
std::pair<nlohmann::json, bool> HdrDriver::GetJointTrajBuffAvail() const {
  return CallApi("/project/robot/trajectory/joint_traject_buf_avail",
                 [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}
/**
 * @return A pair containing the JSON response and a boolean indicating success (true if HTTP 200).
 *
 * @brief Initialize the joint trajectory buffer.
 *
 * @details
 * Clears the trajectory buffer before requesting new trajectories or after motion errors.
 * Must be called when robot is stopped - calling during motion will immediately stop the robot.
 *
 * Typical usage:
 * - Before sending first trajectory when robot is stopped
 * - After error recovery: error occurs → robot stops → init buffer → send new trajectory
 *
 * @warning Calling during trajectory execution will clear buffer and stop robot with error.
 *
 * Supported version: 61.00-00 or higher
 *
 * @see
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/7-joint_traject_init
 */
std::pair<nlohmann::json, bool> HdrDriver::PostInitJointTrajectory() const {
  return CallApi("/project/robot/trajectory/joint_traject_init",
                 [this](const std::string& endpoint) {
                   return api_client_->Post(endpoint, "", nlohmann::json::object());
                 });
}

/**
 * @param trajectories JSON object containing joint trajectory points to execute
 *
 * @return A pair containing the JSON response and a boolean indicating success (true if HTTP 200).
 *
 * @brief Insert joint trajectory points into the controller buffer for motion execution.
 *
 * @details
 * Sends multiple trajectory points to the robot controller's internal buffer (max 2048 points).
 * Points are retained until executed and robot reaches target positions.
 *
 * Requirements:
 * - Robot program must be running (e.g., job with "wait di1" in auto mode)
 * - Minimum 2 points required per trajectory
 * - First point's time_from_start must be 0.0 when starting from stop
 * - Each point needs: positions (radians) and time_from_start (seconds)
 *
 * Trajectory format:
 * @code{.json}
 * {
 *   "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
 *   "points": {
 *     "point_1": {
 *       "positions": [0, 1.570796, 0, 0, 0, 0],
 *       "time_from_start": 0.0
 *     },
 *     "point_2": {
 *       "positions": [0.05, 1.570796, 0, 0, 0, 0],
 *       "time_from_start": 0.4
 *     }
 *   }
 * }
 * @endcode
 *
 * Usage patterns:
 * 1. Discontinuous motion (stop between trajectories):
 *    - traj2.point_1.positions must equal traj1.point_n.positions
 *    - traj2.point_1.time_from_start = 0.0
 *
 * 2. Continuous motion (smooth transition):
 *    - traj2.point_1.time_from_start = traj1.point_n.time_from_start + Δ (Δ > 0)
 *    - traj2.point_1.positions must be reachable from traj1.point_n in Δ time
 *
 * @warning Check buffer availability with GetJointTrajBuffAvail() before sending.
 *          Exceeding axis velocity limits may trigger E159 error and stop robot.
 *
 * Common errors (403 response):
 * - E01554: Program not running
 * - ERR_TOO_FEW_POINTS (-6): Less than 2 points
 * - ERR_TOO_MANY_POINTS (-7): More than 2048 points
 * - ERR_POINTS_EXCEED_BUFFER (-8): Exceeds available buffer space
 *
 * Supported version: 61.00-00 or higher
 *
 * @see
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/8-joint_traject_insert_points
 */
std::pair<nlohmann::json, bool> HdrDriver::PostInsertJointTrajectoryPoints(
    const nlohmann::json& trajectories) const {

  if (!trajectories.is_object()) {
    return {nlohmann::json{{"error", "Invalid JSON: root must be an object"}}, false};
  }
  if (!trajectories.contains("joint_names") || !trajectories["joint_names"].is_array()) {
    return {nlohmann::json{{"error", "Missing or invalid 'joint_names'"}}, false};
  }
  if (!trajectories.contains("points")) {
    return {nlohmann::json{{"error", "Missing 'points'"}}, false};
  }

  const auto& jnames = trajectories["joint_names"];
  const std::size_t dof = jnames.size();
  if (dof == 0) {
    return {nlohmann::json{{"error", "No joint_names provided"}}, false};
  }

  nlohmann::json points_obj = nlohmann::json::object();
  const auto& pts = trajectories["points"];

  if (pts.is_object()) {
    points_obj = pts;
  } else if (pts.is_array()) {
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const std::string key = "point_" + std::to_string(i + 1);
      points_obj[key] = pts[i];
    }
  } else {
    return {nlohmann::json{{"error", "Invalid 'points': must be object or array"}}, false};
  }

  if (points_obj.size() < 2) {
    return {nlohmann::json{{"error", "ERR_TOO_FEW_POINTS: Need at least 2 points"}}, false};
  }

  double prev_tfs = -1.0;
  std::size_t idx = 0;
  for (const auto& kv : points_obj.items()) {
    ++idx;
    const auto& pt = kv.value();
    const std::string pkey = kv.key();

    if (!pt.is_object()) {
      return {nlohmann::json{{"error", "Invalid point: " + pkey + " must be an object"}}, false};
    }
    if (!pt.contains("positions") || !pt["positions"].is_array()) {
      return {nlohmann::json{{"error", "Missing/invalid 'positions' at " + pkey}}, false};
    }
    if (!pt.contains("time_from_start") || !pt["time_from_start"].is_number()) {
      return {nlohmann::json{{"error", "Missing/invalid 'time_from_start' at " + pkey}}, false};
    }

    const auto& pos = pt["positions"];
    if (pos.size() != dof) {
      return {nlohmann::json{{"error", "positions size mismatch at " + pkey + " (expected " +
                                           std::to_string(dof) + ")"}},
              false};
    }

    for (std::size_t j = 0; j < pos.size(); ++j) {
      if (!pos[j].is_number()) {
        return {nlohmann::json{
                    {"error", "positions[" + std::to_string(j) + "] not a number at " + pkey}},
                false};
      }
    }

    const double tfs = pt["time_from_start"].get<double>();
    if (tfs < 0.0) {
      return {nlohmann::json{{"error", "time_from_start < 0 at " + pkey}}, false};
    }

    if (prev_tfs >= 0.0 && tfs <= prev_tfs) {
      return {nlohmann::json{{"error", "time_from_start must be increasing at " + pkey}}, false};
    }
    prev_tfs = tfs;
  }

  nlohmann::json payload;
  payload["joint_names"] = jnames;
  payload["points"] = points_obj;

  return CallApi("/project/robot/trajectory/joint_traject_insert_points",
                 [this, &payload](const std::string& endpoint) {
                   return api_client_->Post(endpoint, "", payload);
                 });
}

}  // namespace hdrcl
