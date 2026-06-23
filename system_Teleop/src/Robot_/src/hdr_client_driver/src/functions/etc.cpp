#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {

/**
 * @return A pair:
 * - JSON response containing the current time
 * - `true` if successful (HTTP 200), `false` otherwise
 *
 * @brief Get the current system date and time from the robot controller.
 *
 * @details
 * This function queries the robot's internal clock and returns the system time
 * in the following format:
 * - year
 * - mon
 * - day
 * - hour
 * - min
 * - sec
 * Example response:
 * ```json
 * {
 *   "year": 2025,
 *   "mon": 4,
 *   "day": 11,
 *   "hour": 15,
 *   "min": 30,
 *   "sec": 45
 * }
 * ```
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/11-etc/1-clock/1-get/1-date_time
 *
 */

std::pair<nlohmann::json, bool> HdrDriver::GetDateTime() const {
  return CallApi("/clock/date_time",
                 [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}

/**
 * @param year Year (e.g., 2025)
 * @param mon Month (1–12)
 * @param day Day (1–31)
 * @param hour Hour (0–23)
 * @param min Minute (0–59)
 * @param sec Second (0–59)
 *
 * @return A pair:
 * - JSON response
 * - `true` if successful (HTTP 200), `false` otherwise
 *
 * @brief Set the robot controller's system date and time.
 *
 * @details
 * This function updates the internal system time of the robot controller.
 * It validates the input using `std::tm` normalization before sending the request.
 * Example request:
 * ```json
 * {
 *   "year": 2025,
 *   "mon": 4,
 *   "day": 11,
 *   "hour": 15,
 *   "min": 30,
 *   "sec": 45
 * }
 * ```
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/11-etc/1-clock/2-put/1-date_time
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PutDateTime(int year, int month, int day, int hour,
                                                       int minute, int second) const {
  try {
    std::tm tm = {};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = second;

    std::mktime(&tm);

    if (tm.tm_year != year - 1900 || tm.tm_mon != month - 1 || tm.tm_mday != day ||
        tm.tm_hour != hour || tm.tm_min != minute || tm.tm_sec != second) {
      return {nlohmann::json{{"error", "Invalid date/time values provided"}}, false};
    }

    nlohmann::json param = {{"year", year}, {"mon", month},  {"day", day},
                            {"hour", hour}, {"min", minute}, {"sec", second}};

    return CallApi("/clock/date_time", [this, &param](const std::string& endpoint) {
      return api_client_->Put(endpoint, "", param);
    });
  } catch (const std::exception& e) {
    return {nlohmann::json{{"error", "Date Time Error: " + std::string(e.what())}}, false};
  }
}

/**
 * @param n_item Number of log entries to retrieve (must be > 0)
 * @param cat_p Log categories (e.g., "E,W,N")
 * @param id_min Optional minimum log ID
 * @param id_max Optional maximum log ID
 * @param ts_min Optional start timestamp (format: "YYYY/MM/DD HH:mm:ss.SSS")
 * @param ts_max Optional end timestamp (format: "YYYY/MM/DD HH:mm:ss.SSS")
 *
 * @return A pair:
 * - JSON array of logs
 * - `true` if successful (HTTP 200), `false` otherwise
 *
 * @brief Retrieve logs from the controller's log manager with filter options.
 *
 * @details
 * This function allows log filtering based on:
 * - `n_item` (required): number of items to retrieve (must be > 0)
 * - `cat_p` (required): category flags (E,W,N,S,O,I,P,H,C,M)
 *   - E: Error
 *   - W: Warning
 *   - N: Notice
 *   - S: Start/Stop
 *   - O: User Operation
 *   - I: I/O (relay value)
 *   - P: Periodic State
 *   - H: History
 *   - C: Console Output
 *   - M: Miscellaneous
 * - `id_min` (optional): Minimum event ID filter.
 *   - All events have a unique event ID (eid) in the range 0 ~ 0xffffffffffffffff.
 *   - If you pass `id_min` as the maximum of previously received IDs + 1, you can filter out past
 * events and only retrieve new ones.
 *   - If the controller rolls over at 0xffffffffffffffff, the ID resets to 0. Events with small IDs
 * (e.g. 0, 1, 2) are still treated as new and not filtered out.
 * - `id_max` (optional): Maximum event ID filter.
 * - `ts_min` / `ts_max` (optional): Timestamp range filter
 *   - Format: YYYY/MM/DD HH:mm:ss.SSS
 *   - Example: 2023/11/20 18:50:30.955
 * Example query:
 * ```json
 * {
 *   "n_item": 50,
 *   "cat_p": "E,W",
 *   "id_min": 0,
 *   "id_max": 0,
 *   "ts_min": "2025/04/01 00:00:00.000",
 *   "ts_max": "2025/04/10 23:59:59.999"
 * }
 * ```
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/7-log_manager/1-get/1-search
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetLogManager(
    uint64_t n_item, const std::string& category_pattern, std::optional<uint64_t> id_min,
    std::optional<uint64_t> id_max, std::optional<std::string> timestamp_min,
    std::optional<std::string> timestamp_max) const {
  std::vector<std::string> params;

  if (n_item <= 0) {
    return {nlohmann::json{{"error", "Invalid n_item value: must be positive"}}, false};
  }
  params.push_back("n_item=" + std::to_string(n_item));

  if (category_pattern.empty()) {
    return {nlohmann::json{{"error", "cat_p is required and cannot be empty"}}, false};
  }

  {
    std::string valid_categories = "EWNSOIPHCM";
    std::istringstream ss(category_pattern);
    std::string category;
    bool valid = true;

    while (std::getline(ss, category, ',')) {
      if (category.length() != 1 || valid_categories.find(category) == std::string::npos) {
        valid = false;
        break;
      }
    }

    if (!valid) {
      return {nlohmann::json{
                  {"error", "Invalid cat_p format: must be combination of E,W,N,S,O,I,P,H,C,M"}},
              false};
    }

    params.push_back("cat_p=" + util::UrlEncode(category_pattern));
  }

  if (id_min.has_value()) {
    if (id_min.value() > 0xFFFFFFFFFFFFFFFFULL) {
      return {nlohmann::json{{"error", "Invalid id_min: value must be <= 0xffffffffffffffff"}},
              false};
    }
    params.push_back("id_min=" + std::to_string(id_min.value()));
  }

  if (id_max.has_value()) {
    if (id_max.value() > 0xFFFFFFFFFFFFFFFFULL) {
      return {nlohmann::json{{"error", "Invalid id_max: value must be <= 0xffffffffffffffff"}},
              false};
    }
    params.push_back("id_max=" + std::to_string(id_max.value()));
  }

  std::regex timestamp_pattern(R"(\d{4}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}\.\d{3})");

  if (timestamp_min.has_value()) {
    if (!std::regex_match(timestamp_min.value(), timestamp_pattern)) {
      return {nlohmann::json{{"error", "Invalid ts_min format: must be YYYY/MM/DD HH:mm:ss.SSS"}},
              false};
    }
    params.push_back("ts_min=" + util::UrlEncode(timestamp_min.value()));
  }

  if (timestamp_max.has_value()) {
    if (!std::regex_match(timestamp_max.value(), timestamp_pattern)) {
      return {nlohmann::json{{"error", "Invalid ts_max format: must be YYYY/MM/DD HH:mm:ss.SSS"}},
              false};
    }
    params.push_back("ts_max=" + util::UrlEncode(timestamp_max.value()));
  }

  std::string query_params;
  for (size_t i = 0; i < params.size(); ++i) {
    if (i > 0)
      query_params += "&";
    query_params += params[i];
  }

  return CallApi(
      "/logManager/search",
      [this](const std::string& endpoint, const std::string& query) {
        auto [posts, status_code] = api_client_->GetStr(endpoint, query);

        if (posts.empty()) {
          return std::make_pair(nlohmann::json{{"error", "Empty response from server"}}, 500);
        }

        try {
          return std::make_pair(nlohmann::json::parse(posts), status_code);
        } catch (...) {
          std::vector<nlohmann::json> json_objects;
          std::istringstream ss(posts);
          std::string line;

          while (std::getline(ss, line)) {
            if (!line.empty()) {
              try {
                json_objects.emplace_back(nlohmann::json::parse(line));
              } catch (const std::exception& inner) {
                return std::make_pair(
                    nlohmann::json{
                        {"error", "Line parse failed: " + std::string(inner.what()), "raw", line}},
                    500);
              }
            }
          }

          if (json_objects.empty()) {
            return std::make_pair(nlohmann::json{{"error", "Multiline parse failed"}}, 500);
          }

          return std::make_pair(nlohmann::json(json_objects), status_code);
        }
      },
      query_params);
}

}  // namespace hdrcl
