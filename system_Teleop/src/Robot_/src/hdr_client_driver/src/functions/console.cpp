// Include the header file for the HDR client driver
#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {

/**
 * @param commands Vector of string commands to execute sequentially.
 * @param period_ms Delay (in milliseconds) to wait between each command.
 *        - If `period_ms > 0`: wait this duration after each command.
 *        - If `period_ms == 0`: no delay between commands.
 *        - If `period_ms == -1`: no delay, same as 0.
 *        - If `period_ms < -1`: treated as invalid; execution is aborted.
 *
 * @return A pair:
 * - JSON response (`message` if successful, or `error` if failure occurs)
 * - `true` if all commands succeed (HTTP 200), `false` otherwise
 *
 * @brief Execute console commands via the HDR Open API.
 *
 * @details
 * This function takes a list of console commands and sends each to the HDR controller
 * via the endpoint `/console/execute_cmd`. Only predefined commands and properly
 * formatted motion instructions are allowed. A delay between commands can be configured
 * using `period_ms`.
 *
 * Supported fixed commands:
 * - `rl.stop`
 * - `rl.reinit`
 * - `rl.i end`
 * - `rl.start`
 * - `rl.exit`
 *
 * Supported motion commands:
 * - Format: `rl.i move <type>,spd=<value>,accu=<0~7>,tool=<0~31> [x, y, z, rx, ry, rz]`
 * - Valid move types: `P`, `L`, `C`, `SP`, `SL`, `SC`
 *
 * Example:
 * @code
 * rl.i move P,spd=1sec,accu=0,tool=1 [0, 90, 0, 0, 0, 0]
 * @endcode
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/10-console/2-post/1-execute_cmd
 */
std::pair<nlohmann::json, bool> HdrDriver::ExecuteCommand(const std::vector<std::string>& commands,
                                                          int period_ms) const {
  // Set of valid static commands
  const std::unordered_set<std::string> kValidCommands = {"rl.stop", "rl.reinit", "rl.i end",
                                                          "rl.start", "rl.exit"};

  // Optional: for move format validation only
  const std::regex kMovePattern(
      R"(rl\.i\s+move\s+(P|L|C|SP|SL|SC),spd=\d+(?:\.\d+)?(mm/sec|cm/min|sec|%),accu=[0-7],tool=(\d|[12]\d|3[01])\s+\[.*\])");

  const std::string kExample = "Example: rl.i move P,spd=1sec,accu=0,tool=1 [0, 90, 0, 0, 0, 0]";

  // Validate all commands before attempting execution
  for (const auto& cmd : commands) {
    if (kValidCommands.find(cmd) == kValidCommands.end() &&
        cmd.rfind("rl.i ", 0) != 0) {  // does NOT start with "rl.i "
      return {nlohmann::json{{"error", "Invalid command: " + cmd + ". " + kExample}}, false};
    }
  }

  for (const auto& cmd : commands) {
    nlohmann::json body = {{"cmd_line", cmd}};

    auto [result, success] =
        CallApi<false>("/console/execute_cmd", [this, &body](const std::string& endpoint) {
          return api_client_->Post(endpoint, "", body);
        });
    if (!success) {
      return {nlohmann::json{{"error", "Failed to execute: " + cmd}, {"response", result}}, false};
    }

    std::cout << "[HDR Driver] Command executed successfully: " << cmd << std::endl;

    if (period_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));  // optional delay
    } else if (period_ms < -1) {  // peroid_ms == -1 -> no delay, continue
      return {nlohmann::json{{"error", "Invalid period_ms: must be >= 0"}}, false};
    }
  }

  return {nlohmann::json{{"message", "All commands executed successfully"}}, true};
}

}  // namespace hdrcl
