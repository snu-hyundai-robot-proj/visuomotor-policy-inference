#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {

/**
 * @return The OpenAPI version as a `double` (e.g., 1.01).
 *
 * @throws std::runtime_error if the HTTP request fails or if the result is not a numeric value.
 *
 * @brief Retrieve the version of the HDR Open API supported by the controller.
 *
 * @details
 * Queries the controller via the endpoint `/api_ver` to obtain the OpenAPI version
 * currently supported. This is useful to ensure compatibility between client
 * software and the controller firmware.
 * Example response from controller:
 * @code
 * 1.01
 * @endcode
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/2-version/1-get/1-api_ver
 *
 */
double HdrDriver::GetApiVersion() const {
  auto [json, success] = CallApi(
      "/api_ver", [this](const std::string& endpoint) { return api_client_->Get(endpoint); });

  if (!success) {
    std::string error_msg = "Failed to get API version";
    if (json.contains("error")) {
      error_msg += " - " + json["error"].get<std::string>();
    }
    throw std::runtime_error(error_msg);
  }

  try {
    return json.get<double>();
  } catch (const nlohmann::json::exception& e) {
    throw std::runtime_error("Invalid API version response format: " + std::string(e.what()));
  }
}

/**
 * @return The numeric version of the `com` module as a `double` (e.g., 60.29).
 *
 * @throws std::runtime_error if the module list is malformed or the version is missing/unreadable.
 *
 * @brief Retrieve the system version of the communication module (`com`) from the controller.
 *
 * @details
 * Sends a request to `/versions/sysver` to obtain a list of system modules and their versions.
 * This function specifically searches for the version of the module named `"com"`.
 * Example response:
 * @code
 * {
 *   "modules": [
 *     {
 *       "name": "com",
 *       "ver": "60.29-02.dev"
 *     }
 *   ]
 * }
 * @endcode
 * Only the numeric portion of the version is parsed and returned (e.g., `"60.29"`).
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/2-version/1-get/2-sysver
 *
 */
double HdrDriver::GetSysVersion() const {
  auto [json, success] = CallApi("/versions/sysver", [this](const std::string& endpoint) {
    return api_client_->Get(endpoint);
  });

  if (!success) {
    std::string error_msg = "Failed to get system version";
    if (json.contains("error")) {
      error_msg += " - " + json["error"].get<std::string>();
    }
    throw std::runtime_error(error_msg);
  }

  if (!json.contains("modules") || !json["modules"].is_array()) {
    throw std::runtime_error("Invalid response format: missing 'modules' array");
  }

  for (const auto& module : json["modules"]) {
    if (module.contains("name") && module["name"] == "com" && module.contains("ver") &&
        module["ver"].is_string()) {
      std::string version_str = module["ver"].get<std::string>();
      size_t hyphen = version_str.find('-');
      if (hyphen != std::string::npos) {
        version_str = version_str.substr(0, hyphen) + version_str.substr(hyphen + 1);
      }

      try {
        return std::stod(version_str);
      } catch (const std::exception& e) {
        throw std::runtime_error("Cannot convert version to number: '" + version_str + "' - " +
                                 e.what());
      }
    }
  }

  throw std::runtime_error("COM module not found in system version response");
}

}  // namespace hdrcl
