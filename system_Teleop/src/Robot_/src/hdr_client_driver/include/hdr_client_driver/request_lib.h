#pragma once

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

namespace request {

/**
 * @brief A high-performance HTTP client library wrapping libcurl.
 *
 * @details
 * This class provides a simplified interface for making HTTP requests
 * with connection pooling, rate limiting, and automatic retries.
 *
 * @class RequestLib
 */
class RequestLib {
 public:
  /**
   * @param host Base URL for all requests
   *
   * @brief Constructor.
   *
   */
  explicit RequestLib(std::string host);

  /**
   * @brief Destructor.
   *
   */
  ~RequestLib();

  /**
   * @param endpoint API endpoint path
   * @param params URL query parameters
   *
   * @return Pair of JSON response and HTTP status code
   *
   * @brief Perform HTTP GET request and return JSON response.
   *
   */
  std::pair<nlohmann::json, int> Get(const std::string& endpoint, const std::string& params = "");

  /**
   * @param endpoint API endpoint path
   * @param params URL query parameters
   *
   * @return Pair of string response and HTTP status code
   *
   * @brief Perform HTTP GET request and return string response.
   *
   */
  std::pair<std::string, int> GetStr(const std::string& endpoint, const std::string& params = "");

  /**
   * @param endpoint API endpoint path
   * @param params URL query parameters
   * @param body JSON request body
   *
   * @return Pair of JSON response and HTTP status code
   *
   * @brief Perform HTTP POST request with JSON body.
   *
   */
  std::pair<nlohmann::json, int> Post(const std::string& endpoint, const std::string& params,
                                      const nlohmann::json& body);

  /**
   * @param endpoint API endpoint path
   * @param params URL query parameters
   * @param file_content File data to upload
   *
   * @return Pair of JSON response and HTTP status code
   *
   * @brief Perform HTTP POST request with file content.
   *
   */
  std::pair<nlohmann::json, int> PostFile(const std::string& endpoint, const std::string& params,
                                          const std::string& file_content);

  /**
   * @param endpoint API endpoint path
   * @param params URL query parameters
   * @param body JSON request body
   *
   * @return Pair of JSON response and HTTP status code
   *
   * @brief Perform HTTP PUT request with JSON body.
   *
   */
  std::pair<nlohmann::json, int> Put(const std::string& endpoint, const std::string& params,
                                     const nlohmann::json& body);

  /**
   * @param endpoint API endpoint path
   * @param params URL query parameters
   *
   * @return Pair of string response and HTTP status code
   *
   * @brief Perform HTTP DELETE request.
   *
   */
  std::pair<std::string, int> Delete(const std::string& endpoint, const std::string& params = "");

 private:
  // Forward declaration of implementation class (PIMPL pattern)
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace request