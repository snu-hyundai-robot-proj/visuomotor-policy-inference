#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {

/**
 * @param name Relay name string
 * @param start_index Bit-based start index
 * @param length Bit length
 *
 * @return std::pair<json, bool> - Response data and success flag
 *
 * @brief Retrieves relay values from the Hi6 PLC using relay name and bit index range.
 *
 * @details
 * Supported formats:
 *   - Full format: "FB{index}.{relay_type}" or "FN{index}.{relay_type}"
 *       → Allowed relay types: "di", "do", "x", "y" only
 *       → Example: "FB0.DO", "FN1.X"
 *   - Simple format: "{relay_type}"
 *       → Allowed relay types: "si", "so", "m", "s", "r", "k"
 *       → Examples: "M", "S", "SI", "SO"
 * Disallowed:
 *   - "M0", "R100" (invalid simple format)
 *   - "FB0.M", "FN1.SI" (invalid full format relay type)
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetRelayValue(const std::string& name, int start_index,
                                                         int length) const {
  const std::set<std::string> kAllTypes = {"di", "do", "x", "y", "m", "s", "r", "k"};

  const std::set<std::string> kFbFnAllowedTypes = {"di", "do", "x", "y"};

  const std::map<std::string, int> kRelayTotalBits = {
      {"di", 960}, {"do", 960}, {"x", 960}, {"y", 960},    {"si", 960},
      {"so", 960}, {"r", 960},  {"k", 960}, {"m", 160000}, {"s", 160000}};

  const std::map<std::string, std::pair<int, int>> kObjIndexRanges = {
      {"fb", {util::kFbNoMin, util::kFbNoMax}}, {"fn", {util::kFnNoMin, util::kFnNoMax}}};

  std::regex pattern_full(R"(^(fb|fn)(\d+)\.([a-zA-Z]{1,3})$)", std::regex::icase);
  std::regex pattern_simple(R"(^(di|do|si|so|x|y|m|s|r|k)$)", std::regex::icase);
  std::smatch matches;

  std::string obj_type, relay_type;
  int obj_idx = -1;
  bool pattern_full_matched = false;

  // 1. Parse relay name
  if (std::regex_match(name, matches, pattern_full)) {
    pattern_full_matched = true;
    obj_type = util::ToLower(matches[1].str());
    obj_idx = std::stoi(matches[2].str());
    relay_type = util::ToLower(matches[3].str());

    if (kObjIndexRanges.count(obj_type)) {
      auto [min_idx, max_idx] = kObjIndexRanges.at(obj_type);
      if (obj_idx < min_idx || obj_idx > max_idx) {
        return {nlohmann::json{{"error", "Object index out of range: " + std::to_string(obj_idx)}},
                false};
      }
    }
  } else if (std::regex_match(name, matches, pattern_simple)) {
    relay_type = util::ToLower(matches[1].str());
  } else {
    return {nlohmann::json{{"error", "Invalid relay name format. Use 'FB0.DO' or 'M'."}}, false};
  }

  // 2. Check if relay_type is supported
  if (!kAllTypes.count(relay_type)) {
    return {nlohmann::json{{"error", "Unsupported relay type: " + relay_type}}, false};
  }

  // 3. Full format restriction: only di/do/x/y
  if (pattern_full_matched && !kFbFnAllowedTypes.count(relay_type)) {
    return {
        nlohmann::json{{"error", "Relay type '" + relay_type + "' is not allowed with " + obj_type +
                                     ". Only di, do, x, y are supported in FB/FN format."}},
        false};
  }

  // 4. Simple format restriction: di/do/x/y not allowed in simple format
  if (!pattern_full_matched && kFbFnAllowedTypes.count(relay_type)) {
    return {nlohmann::json{
                {"error", "Relay type '" + relay_type + "' is only allowed in FB/FN format."}},
            false};
  }

  // 5. Bit range validation
  if (!kRelayTotalBits.count(relay_type)) {
    return {nlohmann::json{{"error", "No size definition for relay type: " + relay_type}}, false};
  }

  int max_bits = kRelayTotalBits.at(relay_type);
  if (start_index < 0 || length < 1 || (start_index + length) > max_bits) {
    return {nlohmann::json{{"error", "Invalid bit range for " + relay_type +
                                         ": st=" + std::to_string(start_index) +
                                         ", len=" + std::to_string(length) +
                                         ", max=" + std::to_string(max_bits)}},
            false};
  }

  // 6. Compose URL
  std::string url;
  if (pattern_full_matched) {
    url = "/project/plc/" + obj_type + std::to_string(obj_idx) + "_" + relay_type + "/val_s32";
  } else {
    url = "/project/plc/" + relay_type + "/val_s32";
  }

  // 7. Perform GET request
  std::string query_string = "st=" + std::to_string(start_index) + "&len=" + std::to_string(length);
  return CallApi(url, [this, &query_string](const std::string& endpoint) {
    return api_client_->Get(endpoint, query_string);
  });
}

/**
 * @param name The relay name string.
 * @param value The value to be set.
 *
 * @return std::pair<nlohmann::json, bool> result json and success flag.
 *
 * @brief Sets a relay value in the Hi6 robot controller's internal PLC.
 *
 * @details
 * This method sends a POST request to the controller to set a specific relay's value.
 * The relay is identified by a name string with one of the following formats:
 *   - `FB{block-index}.{relay-type}{data-type}{signal-index}` (e.g., FB3.DOF12)
 *   - `{relay-type}{data-type}{signal-index}` (e.g., M0, SB128, DOF12)
 * Supported Base Types:
 *   - Logical:     di, do
 *   - System:      si, so
 *   - Physical:    x, y
 *   - Memory:      m, s
 *   - Temporary:   r
 *   - Persistent:  k
 * Supported Data Type Suffixes:
 *   - (none): bit
 *   - b: signed byte (8-bit)
 *   - w: signed word (16-bit)
 *   - l: signed long (32-bit)
 *   - f: float (32-bit)
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::SetRelayValue(const std::string& name,
                                                         double value) const {
  const std::set<std::string> kBaseTypes = {"di", "do", "si", "so", "x", "y", "m", "s", "r", "k"};
  const std::set<std::string> kSuffixes = {"", "b", "w", "l", "f"};

  const std::map<std::string, int> kRelayTotalBits = {
      {"di", 960}, {"do", 960},   {"si", 960},   {"so", 960}, {"x", 960},
      {"y", 960},  {"m", 160000}, {"s", 160000}, {"r", 960},  {"k", 960}};

  const std::map<std::string, int> kSuffixBitSize = {
      {"", 1}, {"b", 8}, {"w", 16}, {"l", 32}, {"f", 32}};

  const std::map<std::string, std::pair<int, int>> kObjIndexRanges = {
      {"fb", {util::kFbNoMin, util::kFbNoMax}}, {"fn", {util::kFnNoMin, util::kFnNoMax}}};

  std::regex pattern_full(R"(^(fb|fn)(\d+)\.([a-zA-Z]{1,3})(\d+)$)", std::regex::icase);
  std::regex pattern_simple(R"(^([a-zA-Z]{1,3})(\d*)$)", std::regex::icase);
  std::smatch matches;

  std::string obj_type, relay_type, base_type, suffix;
  int obj_idx = -1;
  int signal_idx = -1;

  if (std::regex_match(name, matches, pattern_full)) {
    obj_type = util::ToLower(matches[1].str());
    obj_idx = std::stoi(matches[2].str());
    relay_type = util::ToLower(matches[3].str());
    signal_idx = std::stoi(matches[4].str());

    if (kObjIndexRanges.count(obj_type)) {
      auto [min_idx, max_idx] = kObjIndexRanges.at(obj_type);
      if (obj_idx < min_idx || obj_idx > max_idx) {
        return {nlohmann::json{{"error", "Object index out of range for " + obj_type}}, false};
      }
    }
  } else if (std::regex_match(name, matches, pattern_simple)) {
    relay_type = util::ToLower(matches[1].str());
    signal_idx = matches[2].matched ? std::stoi(matches[2].str()) : 0;
  } else {
    return {nlohmann::json{{"error", "Invalid name format. Use 'FB3.DOF12' or 'M0', etc."}}, false};
  }

  // Try all possible splits: base + suffix
  bool valid_type = false;
  for (int i = static_cast<int>(relay_type.length()); i >= 1; --i) {
    base_type = relay_type.substr(0, i);
    suffix = relay_type.substr(i);
    if (kBaseTypes.count(base_type) && kSuffixes.count(suffix)) {
      valid_type = true;
      break;
    }
  }

  if (!valid_type) {
    return {nlohmann::json{{"error", "Invalid relay type or suffix: " + relay_type}}, false};
  }

  if (!kRelayTotalBits.count(base_type) || !kSuffixBitSize.count(suffix)) {
    return {nlohmann::json{{"error", "Unknown relay type or suffix: " + base_type + suffix}},
            false};
  }

  int max_index = kRelayTotalBits.at(base_type) / kSuffixBitSize.at(suffix) - 1;

  if (signal_idx < 0 || signal_idx > max_index) {
    return {nlohmann::json{{"error", "Signal index out of range for " + base_type + suffix + ": " +
                                         std::to_string(signal_idx) +
                                         " (max: " + std::to_string(max_index) + ")"}},
            false};
  }

  nlohmann::json payload = {{"name", name}, {"value", value}};

  return CallApi("/project/plc/set_relay_value", [this, &payload](const std::string& endpoint) {
    return api_client_->Post(endpoint, "", payload);
  });
}

}  // namespace hdrcl