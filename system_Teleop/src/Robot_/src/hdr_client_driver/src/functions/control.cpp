#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {

/**
 * @return A pair:
 * - JSON response with operation conditions
 * - true if HTTP 200
 *
 * @brief Retrieve the current operation condition settings.
 *
 * @details
 * Sends a GET request to retrieve the robot controller's execution condition configuration.
 * Returns information such as:
 * - `playback_mode` (bool)
 * - `step_goback_max_spd` (double)
 * - `ucrd_num` (int)
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/1-get/1-op_cnd
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetControlOpCnd() const {
  return CallApi("/project/control/op_cnd",
                 [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}

/**
 * @param type     The type of the digital I/O signal (e.g., "di", "dob").
 * @param blk_no   The block number (must be between util::kFbNoMin and util::kFbNoMax).
 * @param sig_no   The signal number within the block.
 *
 * @return std::pair<IoResponse, bool>
 *         - IoResponse: contains the result value and a status message.
 *         - bool: true if the request was successful (HTTP 200), false otherwise.
 *
 * @brief Reads the value of a specific digital I/O (DI or DO) signal.
 *
 * @details
 * This function determines the appropriate endpoint based on the I/O type and
 * sends a GET request to read the signal value.
 * Supported types:
 * - Digital Input (DI): "di", "dib", "diw", "dil", "dif"
 * - Digital Output (DO): "do", "dob", "dow", "dol", "dof"
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/6-io_plc/1-get/2-ios-dio
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetControlIosDio(const std::string& raw_type, int blk_no,
                                                            int sig_no) const {
  std::string type = util::ToLower(raw_type);

  if (type.size() < 2)
    return {nlohmann::json{{"error", "Invalid IO type: " + raw_type}}, false};

  const std::string prefix = type.substr(0, 2);
  const std::string suffix = (type.size() > 2) ? type.substr(2) : "";

  std::string endpoint;
  if (prefix == "di")
    endpoint = "di_val";
  else if (prefix == "do")
    endpoint = "do_val";
  else
    return {nlohmann::json{{"error", "Invalid IO prefix: " + prefix}}, false};

  int bit_size = util::GetBitSizeBySuffix(suffix);
  if (bit_size <= 0)
    return {nlohmann::json{{"error", "Unknown suffix: " + suffix}}, false};

  const int sig_max = (util::kFnNoMax / bit_size) - 1;

  if (blk_no < util::kFbNoMin || blk_no > util::kFbNoMax)
    return {nlohmann::json{{"error", "Block number out of range: " + std::to_string(blk_no)}},
            false};

  if (sig_no < util::kSigNoMin || sig_no > sig_max)
    return {nlohmann::json{{"error", "Signal number out of range: " + std::to_string(sig_no)}},
            false};

  const std::string params =
      "type=" + type + "&blk_no=" + std::to_string(blk_no) + "&sig_no=" + std::to_string(sig_no);

  return CallApi(
      "/project/control/ios/dio/" + endpoint,
      [this](const std::string& ep, const std::string& qs) { return api_client_->Get(ep, qs); },
      params);
}

/**
 * @param type The signal type string. Valid types include digital and analog input/output
 * identifiers.
 * @param sig_no The signal number to query (must be >= util::sig_no_min).
 *
 * @return std::pair<IoResponse, bool>
 *         - IoResponse: structure containing status, message, and the retrieved value.
 *         - bool: success status of the HTTP request and response parsing.
 *
 * @brief Retrieve the value of a signal from the robot's SIO (Special I/O).
 *
 * @details
 * This function fetches the current value of a digital or analog special input/output signal
 * from the robot controller using the HTTP GET API.
 * It supports the following types:
 *   - Inputs:  "si", "sib", "siw", "sil", "sif"
 *   - Outputs: "so", "sob", "sow", "sol", "sof"
 *
 * @see(https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/6-io_plc/1-get/3-ios-sio)
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetControlIosSio(const std::string& raw_type,
                                                            int sig_no) const {
  std::string type = util::ToLower(raw_type);
  if (type.size() < 2)
    return {nlohmann::json{{"error", "Invalid IO type: " + raw_type}}, false};

  const std::string prefix = type.substr(0, 2);
  const std::string suffix = (type.size() > 2) ? type.substr(2) : "";

  std::string endpoint;
  if (prefix == "si")
    endpoint = "si_val";
  else if (prefix == "so")
    endpoint = "so_val";
  else
    return {nlohmann::json{{"error", "Invalid IO prefix: " + prefix}}, false};

  int bit_size = util::GetBitSizeBySuffix(suffix);
  if (bit_size <= 0)
    return {nlohmann::json{{"error", "Unknown suffix: " + suffix}}, false};

  const int sig_max = (util::kSigNoMax / bit_size) - 1;
  if (sig_no < util::kSigNoMin || sig_no > sig_max)
    return {nlohmann::json{{"error", "Signal number out of range: " + std::to_string(sig_no)}},
            false};

  const std::string params = "type=" + type + "&sig_no=" + std::to_string(sig_no);

  return CallApi(
      "/project/control/ios/sio/" + endpoint,
      [this](const std::string& ep, const std::string& qs) { return api_client_->Get(ep, qs); },
      params);
}
/**
 * @return A pair:
 * - JSON array with UCS numbers
 * - true if HTTP 200
 *
 * @brief Retrieve the list of available user coordinate system (UCS) numbers.
 *
 * @details
 * Sends a GET request to retrieve available user coordinate indices used for motion programming.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/1-get/4-ucss-ucs_nos
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetControlUcsNos() const {
  return CallApi("/project/control/ucss/ucs_nos",
                 [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}

/**
 * @param type DO type
 * @param blk_no Block number (0 ~ 9)
 * @param sig_no Signal number
 * @param val Value to set (usually 0 or 1)
 *
 * @return A pair:
 * - IoResponse with result
 * - true if request was successful (HTTP 200)
 *
 * @brief Set the value of a digital output (DO) signal.
 *
 * @details
 * Supported types: `do`, `dob`, `dow`, `dol`, `dof`.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/6-io_plc/2-post/2-ios-dio
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostControlIosDio(const std::string& raw_type,
                                                             int blk_no, int sig_no,
                                                             int val) const {
  static const std::set<std::string> kAllowedTypes = {"di",  "do",  "dib", "dob", "diw",
                                                      "dow", "dil", "dol", "dif", "dof"};

  std::string type = util::ToLower(raw_type);
  if (kAllowedTypes.find(type) == kAllowedTypes.end())
    return {nlohmann::json{{"error", "Invalid request type: " + raw_type}}, false};

  const std::string suffix = (type.size() > 2) ? type.substr(2) : "";

  int bit_size = util::GetBitSizeBySuffix(suffix);
  if (bit_size <= 0)
    return {nlohmann::json{{"error", "Unknown suffix: " + suffix}}, false};

  const int sig_max = (util::kSigNoMax / bit_size) - 1;
  if (sig_no < util::kSigNoMin || sig_no > sig_max)
    return {nlohmann::json{{"error", "Signal number out of range: " + std::to_string(sig_no)}},
            false};

  if (blk_no < util::kFbNoMin || blk_no > util::kFbNoMax)
    return {nlohmann::json{{"error", "Block number out of range: " + std::to_string(blk_no)}},
            false};

  nlohmann::json body = {{"type", raw_type}, {"blk_no", blk_no}, {"sig_no", sig_no}, {"val", val}};

  return CallApi(
      "/project/control/ios/dio/do_val",
      [this](const std::string& endpoint, const nlohmann::json& b) {
        return api_client_->Post(endpoint, "", b);
      },
      body);
}

/**
 * @param playback_mode Whether playback mode is enabled
 * @param step_goback_max_spd Max speed value for reverse motion
 * @param ucrd_num User coordinate number
 *
 * @return A pair:
 * - JSON response
 * - true if successful (HTTP 200)
 *
 * @brief Update operation condition parameters on the controller.
 *
 * @details
 * Sends a PUT request to update the robot's execution mode and limits.
 * Parameters include:
 * - `playback_mode`: enable/disable playback mode
 * - `step_goback_max_spd`: max speed for step-back motion
 * - `ucrd_num`: user coordinate system number
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/3-put/1-op_cnd
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PutControlOpCnd(bool playback_mode,
                                                           double step_goback_max_spd,
                                                           int ucrd_num) const {
  nlohmann::json body = {{"playback_mode", playback_mode},
                         {"step_goback_max_spd", step_goback_max_spd},
                         {"ucrd_num", ucrd_num}};

  return CallApi("/project/control/op_cnd", [this, &body](const std::string& endpoint) {
    return api_client_->Put(endpoint, "", body);
  });
}
}  // namespace hdrcl
