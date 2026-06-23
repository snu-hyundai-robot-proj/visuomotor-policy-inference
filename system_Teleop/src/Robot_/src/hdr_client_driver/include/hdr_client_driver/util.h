#ifndef ROBOT_UTIL_H_
#define ROBOT_UTIL_H_

/*==============================================================================
 *  Utility helpers for robot client-side libraries.
 *
 *  - Unit-conversion constants (deg↔rad)
 *  - Common index limits (fb, fn, signal, tool, task, coordinate)
 *  - Validation utilities for network parameters
 *  - Custom exception classes
 *  - Misc helpers: lowercase conversion, URL builders / encoders, API-response logger
 *============================================================================*/

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>  // External single-header JSON library
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace hdrcl::util {

/*==============================================================================
 *  Custom Exception Classes
 *============================================================================*/

/**
 * @brief Custom exception class for network-related errors.
 *
 * @details
 * Specialized exception for network communication failures, connection
 * timeouts, and socket-related errors in the HDR driver system.
 */
class NetworkException : public std::runtime_error {
 public:
  /**
   * @param message Error message describing the network issue
   *
   * @brief Constructor for network exception.
   */
  explicit NetworkException(const std::string& message)
      : std::runtime_error("Network Error: " + message) {}
};

/**
 * @brief Custom exception class for API-related errors.
 *
 * @details
 * Specialized exception for OpenAPI communication failures, HTTP errors,
 * and API response parsing issues in the HDR driver system.
 */
class ApiException : public std::runtime_error {
 public:
  /**
   * @param message Error message describing the API issue
   * @param status_code HTTP status code (optional, default -1)
   *
   * @brief Constructor for API exception with optional status code.
   */
  explicit ApiException(const std::string& message, int status_code = -1)
      : std::runtime_error("API Error: " + message +
                           (status_code > 0 ? " (HTTP " + std::to_string(status_code) + ")" : "")) {
  }
};

/*==============================================================================
 *  Unit conversion
 *============================================================================*/

/**
 * @name Unit-conversion constants
 * @{
 */
constexpr double kDegToRad = M_PI / 180.0;  // Degrees → radians multiplier
constexpr double kRadToDeg = 180.0 / M_PI;  // Radians → degrees multiplier
/**
 */

/*==============================================================================
 *  Index limits (controller conventions)
 *============================================================================*/

// Function-block (FB) number range.
constexpr int kFbNoMin = 0;
constexpr int kFbNoMax = 9;

// Function number (FN) range.
constexpr int kFnNoMin = 0;
constexpr int kFnNoMax = 63;

// Signal number range (digital/analogue I/O).
constexpr int kSigNoMin = 0;
constexpr int kSigNoMax = 960;

// Tool number range (tool table).
constexpr int kToolNoMin = 0;
constexpr int kToolNoMax = 31;

// Task number range (multi-task controller).
constexpr int kTaskNoMin = 0;
constexpr int kTaskNoMax = 7;

// Coordinate-system number range (-1 = robot base, 0-3 = user frames).
constexpr int kCrdNoMin = -1;
constexpr int kCrdNoMax = 3;

/*==============================================================================
 *  Validation Functions
 *============================================================================*/

/**
 * @param ip IP address string to check
 *
 * @return True if IP format is valid, false otherwise
 *
 * @brief Check if string matches basic IPv4 format.
 *
 * @details
 * Performs basic IPv4 format validation using regular expressions.
 * Does not perform comprehensive IP validation but catches obvious errors.
 */
[[nodiscard]] inline bool IsValidIpv4Format(const std::string& ip) {
  std::regex ipv4_pattern(
      R"(^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$)");
  return std::regex_match(ip, ipv4_pattern);
}

/**
 * @param hostname Hostname string to check
 *
 * @return True if hostname format is valid, false otherwise
 *
 * @brief Check if string is a valid hostname format.
 *
 * @details
 * Validates hostname according to RFC standards, allowing alphanumeric
 * characters, hyphens, and dots in appropriate positions.
 */
[[nodiscard]] inline bool IsValidHostnameFormat(const std::string& hostname) {
  if (hostname.empty() || hostname.length() > 253) {
    return false;
  }

  std::regex hostname_pattern(
      R"(^[a-zA-Z0-9]([a-zA-Z0-9\-]{0,61}[a-zA-Z0-9])?(\.[a-zA-Z0-9]([a-zA-Z0-9\-]{0,61}[a-zA-Z0-9])?)*$)");
  return std::regex_match(hostname, hostname_pattern);
}

/**
 * @param ip IP address string to validate
 *
 * @brief Validate IP address format and content.
 *
 * @throws std::invalid_argument if IP address is invalid
 *
 * @details
 * Checks if the IP address is not empty and has a valid format.
 * Supports both IPv4 addresses and hostnames, including localhost aliases.
 */
inline void ValidateIpAddress(const std::string& ip) {
  if (ip.empty()) {
    throw std::invalid_argument("IP address cannot be empty");
  }

  // Check for localhost aliases
  if (ip == "localhost" || ip == "127.0.0.1" || ip == "::1") {
    return;  // Valid localhost addresses
  }

  // Validate IPv4 format or hostname
  if (!IsValidIpv4Format(ip) && !IsValidHostnameFormat(ip)) {
    throw std::invalid_argument("Invalid IP address or hostname format: " + ip);
  }
}

/**
 * @param port Port number to validate
 * @param port_name Descriptive name for the port (for error messages)
 *
 * @brief Validate port number range.
 *
 * @throws std::invalid_argument if port is out of valid range
 *
 * @details
 * Ensures port number is within valid range (1-65535) and provides
 * descriptive error messages including the port type.
 */
inline void ValidatePort(int port, const std::string& port_name = "port") {
  if (port <= 0 || port > 65535) {
    throw std::invalid_argument("Invalid " + port_name + ": " + std::to_string(port) +
                                ". Port must be between 1 and 65535");
  }
}

/**
 * @param mode Socket communication mode string
 *
 * @brief Validate socket communication mode.
 *
 * @throws std::invalid_argument if mode is not supported
 *
 * @details
 * Checks if the provided mode is one of the supported socket modes:
 * TCP_CLIENT, TCP_SERVER, UDP (case-insensitive). Also handles "TCP" alias.
 */
inline void ValidateSocketMode(const std::string& mode) {
  if (mode.empty()) {
    throw std::invalid_argument("Socket mode cannot be empty");
  }

  // Convert to uppercase for comparison
  std::string upper_mode = mode;
  std::transform(upper_mode.begin(), upper_mode.end(), upper_mode.begin(), ::toupper);

  // Handle aliases
  if (upper_mode == "TCP") {
    return;  // Will be normalized to TCP_CLIENT
  }

  if (upper_mode != "TCP_CLIENT" && upper_mode != "TCP_SERVER" && upper_mode != "UDP") {
    throw std::invalid_argument("Invalid socket mode: " + mode +
                                ". Valid modes: TCP_CLIENT, TCP_SERVER, UDP, TCP");
  }
}

/*==============================================================================
 *  Suffix and Bit Size Utilities
 *============================================================================*/

/**
 * @return A reference to the static suffix bit size map.
 *
 * @brief Returns a map of suffix to bit size mappings.
 *
 */
inline const std::map<std::string, int>& GetSuffixBitSizeMap() {
  static const std::map<std::string, int> suffix_bit_size = {
      {"", 1}, {"b", 8}, {"w", 16}, {"l", 32}, {"f", 32}};
  return suffix_bit_size;
}

/**
 * @param suffix The suffix string to lookup.
 *
 * @return The bit size or -1 if not found.
 *
 * @brief Gets bit size from a suffix string.
 *
 */
inline int GetBitSizeBySuffix(const std::string& suffix) {
  const auto& map = GetSuffixBitSizeMap();
  auto it = map.find(suffix);
  return it != map.end() ? it->second : -1;
}

/*==============================================================================
 *  Helper functions
 *============================================================================*/

/**
 * @param str Input string.
 *
 * @return Lower-cased copy.
 *
 * @brief Convert a string to lowercase (ASCII-only, locale-independent).
 *
 */
[[nodiscard]] inline std::string ToLower(const std::string& str) {
  std::string lower = str;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower;
}

/**
 * @param value Floating-point value to format.
 *
 * @return Formatted string with trailing zeros removed.
 *
 * @brief Format a double value as string, removing trailing zeros.
 * Examples:
 * @code
 *   FormatDouble(0.001000) == "0.001"
 *   FormatDouble(2.000000) == "2"
 *   FormatDouble(1.500000) == "1.5"
 *   FormatDouble(-1.000000) == "-1"
 * @endcode
 *
 */
[[nodiscard]] inline std::string FormatDouble(double value) {
  std::string str = std::to_string(value);

  // Remove trailing zeros if decimal point exists
  if (str.find('.') != std::string::npos) {
    str.erase(str.find_last_not_of('0') + 1, std::string::npos);
    // Remove decimal point if it's the last character
    if (str.back() == '.') {
      str.pop_back();
    }
  }

  return str;
}

/**
 * @param ip   Controller IP address.
 * @param port Port on which the OpenAPI server listens (default 8888).
 *
 * @return Formatted URL (scheme + host + port).
 *
 * @brief Build the base OpenAPI URL used by robot controllers.
 * Example:
 * @code
 *   auto url = hdrcl::util::MakeOpenApiUrl("192.168.1.150", 8888);
 *   // url == "http://192.168.1.150:8888"
 * @endcode
 *
 * @throws std::invalid_argument if IP address or port is invalid
 */
[[nodiscard]] inline std::string MakeOpenApiUrl(const std::string& ip, int port) {
  // Validate inputs before creating URL
  ValidateIpAddress(ip);
  ValidatePort(port, "OpenAPI port");

  std::string url = "http://" + ip + ":" + std::to_string(port);
  std::cout << "[util] OpenAPI URL: " << url << '\n';
  return url;
}

/**
 * @param value Raw query or path segment.
 *
 * @return URL-escaped string.
 *
 * @brief Percent-encode a string for safe inclusion in URLs (RFC 3986).
 *
 */
[[nodiscard]] inline std::string UrlEncode(const std::string& value) {
  std::ostringstream escaped;
  escaped.fill('0');
  escaped << std::hex << std::uppercase;

  for (unsigned char c : value) {
    if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      escaped << c;
    } else {
      escaped << '%' << std::setw(2) << static_cast<int>(c);
    }
  }
  return escaped.str();
}

/**
 * @param endpoint    Full endpoint string (e.g. "/project/control/ios/dio/di_val").
 * @param response    Parsed JSON body.
 * @param status_code HTTP status code returned by the request.
 *
 * @brief Pretty-print an API response with color-coded status.
 * Colors use ANSI escape codes and will gracefully fall back to plain text
 * on terminals that do not support color.
 *
 */
inline void LogApiResponse(const std::string& endpoint, const nlohmann::json& response,
                           int status_code) {
  const char* green = "\033[32m";
  const char* yellow = "\033[33m";
  const char* red = "\033[31m";
  const char* reset = "\033[0m";

  const char* color = (status_code >= 200 && status_code < 300)   ? green
                      : (status_code >= 300 && status_code < 400) ? yellow
                                                                  : red;

  std::cout << "\n======= API RESPONSE =======\n"
            << "Endpoint : " << endpoint << '\n'
            << "Status   : " << color << status_code << reset << '\n'
            << "Body     :\n"
            << response.dump(4) << '\n'
            << "============================\n";
}

}  // namespace hdrcl::util

#endif  // ROBOT_UTIL_H_
