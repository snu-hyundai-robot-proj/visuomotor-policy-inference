#include "hdr_client_driver/request_lib.h"

#include <curl/curl.h>

#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>

namespace request {
namespace {

/**
 * @param contents Pointer to the incoming data buffer
 * @param size Size of each element
 * @param nmemb Number of elements
 * @param userp Pointer to the string buffer where data will be appended
 *
 * @return Total number of bytes written
 *
 * @brief Callback function for libcurl to write received data to a string.
 *
 */
inline size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
  auto* dst = static_cast<std::string*>(userp);
  if (!contents || !dst)
    return 0;
  dst->append(static_cast<const char*>(contents), size * nmemb);
  return size * nmemb;
}

/**
 * @param host Base URL
 * @param endpoint API endpoint path
 * @param params URL query parameters
 *
 * @return Complete URL string with optional query parameters
 *
 * @brief Build complete URL by combining host, endpoint, and query parameters.
 *
 */
inline std::string BuildUrl(const std::string& host, const std::string& endpoint,
                            const std::string& params) {
  std::string url = host + endpoint;
  if (!params.empty())
    url += "?" + params;
  return url;
}

//--------------------------------------------------------------------
// Simple rate‑limiter (token bucket ~1 call every min_interval)
//--------------------------------------------------------------------
/**
 * @brief Token bucket rate limiter for controlling request frequency.
 *
 * @details
 * Implements a token bucket algorithm to limit the rate of HTTP requests.
 * Each request consumes one token, and tokens are refilled at a constant rate.
 *
 */
class RateLimiter {
 public:
  /**
   * @param max_calls_per_second Maximum allowed calls per second
   * @param burst_size Maximum number of tokens in the bucket (default 10)
   *
   * @brief Constructor that initializes the rate limiter with specified limits.
   *
   */
  explicit RateLimiter(int max_calls_per_second, int burst_size = 10)
      : tokens_(burst_size),
        max_tokens_(burst_size),
        token_rate_(static_cast<double>(max_calls_per_second) / 1000.0),
        fixed_sleep_us_(1'000'000 / max_calls_per_second),
        last_refill_(std::chrono::steady_clock::now()) {}
  /**
   * @brief Wait for available token before allowing request to proceed.
   *
   * @details
   * Blocks the calling thread until a token is available. Implements both
   * token bucket refilling and fixed minimum delay between requests.
   *
   */
  void Wait() {
    std::lock_guard<std::mutex> lock(mtx_);
    RefillTokens();

    if (tokens_ < 1.0) {
      double wait_time = (1.0 - tokens_) / token_rate_;
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(wait_time)));
      RefillTokens();
    }

    std::this_thread::sleep_for(std::chrono::microseconds(fixed_sleep_us_));

    tokens_ -= 1.0;
  }

 private:
  /**
   * @brief Refill tokens based on elapsed time since last refill.
   *
   */
  void RefillTokens() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_refill_).count();

    if (elapsed > 0) {
      double new_tokens = elapsed * token_rate_;
      tokens_ = std::min(tokens_ + new_tokens, max_tokens_);
      last_refill_ = now;
    }
  }

  double tokens_;                                      ///< Current number of available tokens
  double max_tokens_;                                  ///< Maximum number of tokens in bucket
  double token_rate_;                                  ///< Token refill rate per millisecond
  int fixed_sleep_us_;                                 ///< Fixed sleep duration in microseconds
  std::chrono::steady_clock::time_point last_refill_;  ///< Last token refill timestamp
  std::mutex mtx_;                                     ///< Mutex for thread-safe access
};

//--------------------------------------------------------------------
// Error helpers (JSON / string specialisation)
//--------------------------------------------------------------------
/**
 * @param msg Error message string
 * @param code HTTP status code
 *
 * @return Template specialization for error response
 *
 * @brief Template function for creating error responses (to be specialized).
 *
 */
template <typename T>
T MakeError(const std::string& msg, int code);

/**
 * @param msg Error message string
 * @param code HTTP status code
 *
 * @return Pair containing JSON error object and status code
 *
 * @brief Template specialization for creating JSON error responses.
 *
 */
template <>
inline std::pair<nlohmann::json, int> MakeError(const std::string& msg, int code) {
  return {nlohmann::json{{"error", msg}}, code};
}

/**
 * @param msg Error message string
 * @param code HTTP status code
 *
 * @return Pair containing error message string and status code
 *
 * @brief Template specialization for creating string error responses.
 *
 */
template <>
inline std::pair<std::string, int> MakeError(const std::string& msg, int code) {
  return {msg, code};
}

}  // anonymous namespace

//--------------------------------------------------------------------
// RequestLib::Impl  —  private implementation (PIMPL)
//--------------------------------------------------------------------

/**
 * @brief Private implementation class for RequestLib using PIMPL pattern.
 *
 * @details
 * Contains the actual HTTP client implementation with libcurl integration,
 * rate limiting, and connection management.
 *
 */
class RequestLib::Impl {
 public:
  /**
   * @param host Base URL for all HTTP requests
   *
   * @brief Constructor that initializes libcurl and rate limiter.
   *
   * @throws std::invalid_argument if host URL is empty
   */
  explicit Impl(std::string host) : host_(std::move(host)), limiter_(1000 /*calls/s*/) {
    if (host_.empty())
      throw std::invalid_argument("Host URL cannot be empty");
    curl_global_init(CURL_GLOBAL_DEFAULT);
  }

  /**
   * @brief Destructor that cleans up libcurl global resources.
   *
   */
  ~Impl() { curl_global_cleanup(); }

  // Public wrappers ------------------------------------------------
  /**
   * @param ep API endpoint path
   * @param p URL query parameters
   *
   * @return Pair of JSON response and HTTP status code
   *
   * @brief Perform HTTP GET request and return JSON response.
   *
   */
  std::pair<nlohmann::json, int> GetJson(const std::string& ep, const std::string& p) {
    return Exec<std::pair<nlohmann::json, int>>(
        [&](CURL* h) { return performJson(h, "GET", ep, p, ""); });
  }

  /**
   * @param ep API endpoint path
   * @param p URL query parameters
   *
   * @return Pair of string response and HTTP status code
   *
   * @brief Perform HTTP GET request and return string response.
   *
   */
  std::pair<std::string, int> GetStr(const std::string& ep, const std::string& p) {
    return Exec<std::pair<std::string, int>>(
        [&](CURL* h) { return performStr(h, "GET", ep, p, ""); });
  }

  /**
   * @param ep API endpoint path
   * @param p URL query parameters
   * @param body JSON request body
   *
   * @return Pair of JSON response and HTTP status code
   *
   * @brief Perform HTTP POST request with JSON body.
   *
   */
  std::pair<nlohmann::json, int> PostJson(const std::string& ep, const std::string& p,
                                          const nlohmann::json& body) {
    return Exec<std::pair<nlohmann::json, int>>(
        [&](CURL* h) { return performJson(h, "POST", ep, p, body.dump()); });
  }

  /**
   * @param ep API endpoint path
   * @param p URL query parameters
   * @param bin Binary file content to upload
   *
   * @return Pair of JSON response and HTTP status code
   *
   * @brief Perform HTTP POST request with binary file content.
   *
   */
  std::pair<nlohmann::json, int> PostFile(const std::string& ep, const std::string& p,
                                          const std::string& bin) {
    return Exec<std::pair<nlohmann::json, int>>(
        [&](CURL* h) { return performJson(h, "POST", ep, p, bin, "application/octet-stream"); });
  }

  /**
   * @param ep API endpoint path
   * @param p URL query parameters
   * @param body JSON request body
   *
   * @return Pair of JSON response and HTTP status code
   *
   * @brief Perform HTTP PUT request with JSON body.
   *
   */
  std::pair<nlohmann::json, int> PutJson(const std::string& ep, const std::string& p,
                                         const nlohmann::json& body) {
    return Exec<std::pair<nlohmann::json, int>>(
        [&](CURL* h) { return performJson(h, "PUT", ep, p, body.dump()); });
  }

  /**
   * @param ep API endpoint path
   * @param p URL query parameters
   *
   * @return Pair of string response and HTTP status code
   *
   * @brief Perform HTTP DELETE request and return string response.
   *
   */
  std::pair<std::string, int> DeleteStr(const std::string& ep, const std::string& p) {
    return Exec<std::pair<std::string, int>>(
        [&](CURL* h) { return performStr(h, "DELETE", ep, p, ""); });
  }

 private:
  //----------------------------------------------------------------
  // Generic executor with retry + rate‑limit
  //----------------------------------------------------------------
  /**
   * @param f Function that performs the actual HTTP request with CURL handle
   *
   * @return Result of type R from the function execution
   *
   * @brief Generic request executor with rate limiting and retry logic.
   *
   * @tparam R Return type of the request function
   * @tparam F Function type that takes CURL* and returns R
   */
  template <typename R, typename F>
  R Exec(F&& f) {
    limiter_.Wait();
    constexpr int kRetry = 1;
    constexpr auto kRetryDelay = std::chrono::milliseconds{1};

    for (int i = 0; i < kRetry; ++i) {
      CURL* curl = curl_easy_init();
      if (!curl)
        return MakeError<R>("Failed to init CURL", 500);

      auto ret = f(curl);
      curl_easy_cleanup(curl);

      const int status = std::get<1>(ret);
      if (status >= 200 && status < 300)
        return ret;  // success
      if (i + 1 < kRetry)
        std::this_thread::sleep_for(kRetryDelay);
    }
    return MakeError<R>("All retry attempts failed", 500);
  }

  //----------------------------------------------------------------
  // Concrete request helpers
  //----------------------------------------------------------------
  /**
   * @param curl CURL handle for the request
   * @param method HTTP method string (GET, POST, PUT, DELETE)
   * @param ep API endpoint path
   * @param pr URL query parameters
   * @param body Request body content
   * @param ctype Content-Type header (default: "application/json; charset=utf-8")
   *
   * @return Pair of JSON response and HTTP status code
   *
   * @brief Perform HTTP request and parse response as JSON.
   *
   */
  std::pair<nlohmann::json, int> performJson(
      CURL* curl, const char* method, const std::string& ep, const std::string& pr,
      const std::string& body, const std::string& ctype = "application/json; charset=utf-8") {
    std::string buffer;
    setupCommon(curl, method, ep, pr, body);

    struct curl_slist* hdrs = nullptr;
    hdrs = curl_slist_append(hdrs, ("Content-Type: " + ctype).c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);

    const int status = perform(curl);
    curl_slist_free_all(hdrs);
    if (status < 0)
      return MakeError<std::pair<nlohmann::json, int>>(buffer, -status);

    try {
      return {buffer.empty() ? nlohmann::json::object() : nlohmann::json::parse(buffer), status};
    } catch (const nlohmann::json::parse_error& e) {
      return MakeError<std::pair<nlohmann::json, int>>(e.what(), 500);
    }
  }

  /**
   * @param curl CURL handle for the request
   * @param method HTTP method string (GET, POST, PUT, DELETE)
   * @param ep API endpoint path
   * @param pr URL query parameters
   * @param body Request body content
   *
   * @return Pair of string response and HTTP status code
   *
   * @brief Perform HTTP request and return raw string response.
   *
   */
  std::pair<std::string, int> performStr(CURL* curl, const char* method, const std::string& ep,
                                         const std::string& pr, const std::string& body) {
    std::string buffer;
    setupCommon(curl, method, ep, pr, body);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);

    const int status = perform(curl);
    if (status < 0)
      return MakeError<std::pair<std::string, int>>(buffer, -status);
    return {std::move(buffer), status};
  }

  //----------------------------------------------------------------
  // Curl helpers
  //----------------------------------------------------------------
  /**
   * @param h CURL handle to configure
   * @param method HTTP method string
   * @param ep API endpoint path
   * @param pr URL query parameters
   * @param body Request body content
   *
   * @brief Set up common CURL options for all request types.
   *
   */
  void setupCommon(CURL* h, const char* method, const std::string& ep, const std::string& pr,
                   const std::string& body) {
    curl_easy_setopt(h, CURLOPT_URL, BuildUrl(host_, ep, pr).c_str());
    curl_easy_setopt(h, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(h, CURLOPT_CONNECTTIMEOUT_MS, 5'000L);
    curl_easy_setopt(h, CURLOPT_TIMEOUT_MS, 30'000L);
    curl_easy_setopt(h, CURLOPT_TCP_KEEPALIVE, 1L);
    curl_easy_setopt(h, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(h, CURLOPT_FORBID_REUSE, 0L);
    curl_easy_setopt(h, CURLOPT_FRESH_CONNECT, 0L);

    if (std::strcmp(method, "GET") == 0) {
      // default — nothing extra
    } else if (std::strcmp(method, "POST") == 0) {
      curl_easy_setopt(h, CURLOPT_POST, 1L);
    } else {
      curl_easy_setopt(h, CURLOPT_CUSTOMREQUEST, method);
    }

    if (!body.empty()) {
      curl_easy_setopt(h, CURLOPT_POSTFIELDS, body.c_str());
      curl_easy_setopt(h, CURLOPT_POSTFIELDSIZE, body.size());
    }
  }

  /**
   * @param h CURL handle to perform request with
   *
   * @return HTTP status code (positive) or negated error code (negative)
   *
   * @brief Execute the configured CURL request and return status code.
   *
   */
  int perform(CURL* h) {
    std::string dummy;  // used when caller didn't set WRITEDATA yet
    // if (!curl_easy_getinfo(h, CURLINFO_WRITEDATA, nullptr))
    // curl_easy_setopt(h, CURLOPT_WRITEDATA, &dummy);

    CURLcode rc = curl_easy_perform(h);
    if (rc != CURLE_OK)
      return -((rc == CURLE_OPERATION_TIMEDOUT) ? 408 : 500);

    long code = 0;
    curl_easy_getinfo(h, CURLINFO_RESPONSE_CODE, &code);
    return static_cast<int>(code);
  }

  //----------------------------------------------------------------
  // Members
  //----------------------------------------------------------------
  std::string host_;     ///< Base URL for all requests
  RateLimiter limiter_;  ///< Rate limiter instance
};

//--------------------------------------------------------------------
// RequestLib  —  public interface delegating to Impl
//--------------------------------------------------------------------
/**
 * @param host Base URL for all HTTP requests
 *
 * @brief Constructor that creates and initializes the implementation.
 *
 */
RequestLib::RequestLib(std::string host) : impl_(std::make_unique<Impl>(std::move(host))) {}

/**
 * @brief Destructor (default implementation due to PIMPL pattern).
 *
 */
RequestLib::~RequestLib() = default;

/**
 * @param ep API endpoint path
 * @param p URL query parameters
 *
 * @return Pair of JSON response and HTTP status code
 *
 * @brief Public interface for HTTP GET request returning JSON.
 *
 */
std::pair<nlohmann::json, int> RequestLib::Get(const std::string& ep, const std::string& p) {
  return impl_->GetJson(ep, p);
}

/**
 * @param ep API endpoint path
 * @param p URL query parameters
 *
 * @return Pair of string response and HTTP status code
 *
 * @brief Public interface for HTTP GET request returning string.
 *
 */
std::pair<std::string, int> RequestLib::GetStr(const std::string& ep, const std::string& p) {
  return impl_->GetStr(ep, p);
}

/**
 * @param ep API endpoint path
 * @param p URL query parameters
 * @param b JSON request body
 *
 * @return Pair of JSON response and HTTP status code
 *
 * @brief Public interface for HTTP POST request with JSON body.
 *
 */
std::pair<nlohmann::json, int> RequestLib::Post(const std::string& ep, const std::string& p,
                                                const nlohmann::json& b) {
  return impl_->PostJson(ep, p, b);
}

/**
 * @param ep API endpoint path
 * @param p URL query parameters
 * @param bin Binary file content to upload
 *
 * @return Pair of JSON response and HTTP status code
 *
 * @brief Public interface for HTTP POST request with file content.
 *
 */
std::pair<nlohmann::json, int> RequestLib::PostFile(const std::string& ep, const std::string& p,
                                                    const std::string& bin) {
  return impl_->PostFile(ep, p, bin);
}

/**
 * @param ep API endpoint path
 * @param p URL query parameters
 * @param b JSON request body
 *
 * @return Pair of JSON response and HTTP status code
 *
 * @brief Public interface for HTTP PUT request with JSON body.
 *
 */
std::pair<nlohmann::json, int> RequestLib::Put(const std::string& ep, const std::string& p,
                                               const nlohmann::json& b) {
  return impl_->PutJson(ep, p, b);
}

/**
 * @param ep API endpoint path
 * @param p URL query parameters
 *
 * @return Pair of string response and HTTP status code
 *
 * @brief Public interface for HTTP DELETE request.
 *
 */
std::pair<std::string, int> RequestLib::Delete(const std::string& ep, const std::string& p) {
  return impl_->DeleteStr(ep, p);
}

}  // namespace request