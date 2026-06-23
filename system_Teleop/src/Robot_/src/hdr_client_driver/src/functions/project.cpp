#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {
/**
 * @return A pair consisting of:
 * - `nlohmann::json`: Response containing the RGEN value
 * - `bool`: True if the HTTP status code is 2xx, false otherwise
 *
 * @brief Get the current RGEN value (project execution state).
 *
 * @details
 * Retrieves the current RGEN status which indicates the project execution state.
 * Possible values:
 * - 0: Not running
 * - 1: Running
 * - 2: Paused (if applicable)
 * This sends a GET request to the `/project/rgen` endpoint.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/1-get/1-rgen
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetProjectRgen() const {
  return CallApi<false>("/project/rgen",
                        [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}
/**
 * @return A pair consisting of:
 * - `nlohmann::json`: Array of job metadata
 * - `bool`: True if the request succeeded (HTTP 2xx), false otherwise
 *
 * @brief Retrieve metadata of all jobs registered in the project.
 *
 * @details
 * This sends a GET request to the `/project/jobs_info` endpoint
 * and returns job information such as:
 * - job name
 * - file path
 * - whether the job was modified
 * Example response:
 * ```json
 * {
 *   "jobs": [
 *     {
 *       "name": "main.job",
 *       "path": "/project/main.job",
 *       "modified": false
 *     }
 *   ]
 * }
 * ```
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/1-get/2-jobs_info
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetProjectJobsInfo() const {
  return CallApi("/project/jobs_info",
                 [this](const std::string& endpoint) { return api_client_->Get(endpoint); });
}

/**
 * @return True if the operation succeeded (HTTP 200), false otherwise
 *
 * @brief Reload and synchronize jobs that were modified externally.
 *
 * @details
 * This sends a POST request to `/project/reload_updated_jobs` with an empty body.
 * It updates in-memory job states to reflect the latest file changes.
 * This is useful when jobs are edited by external editors.
 *
 * @see
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/2-post/1-reload_updated_jobs
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostProjectReloadUpdateJobs() const {
  return CallApi("/project/reload_updated_jobs", [this](const std::string& endpoint) {
    return api_client_->Post(endpoint, "", nlohmann::json::object());
  });
}

/**
 * @param path Relative path of the job file to delete (e.g., "subdir/sample.job")
 *
 * @return A pair consisting of:
 * - `nlohmann::json`: Response object from the server
 * - `bool`: True if the job was successfully deleted (HTTP 200), false otherwise
 *
 * @brief Delete a specific job file from the project.
 *
 * @details
 * This sends a POST request to `/project/jobs/delete_job` with the `fname` parameter
 * specifying the relative file path of the job to delete.
 * Example usage:
 * ```
 * post_project_delete_job("main.job");
 * ```
 * If the `path` is empty, the function returns an error response.
 *
 * @see
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/2-post/2-jobs-delete_job
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostProjectDeleteJob(const std::string& path) const {
  if (path.empty()) {
    return {nlohmann::json::object({{"error", "Invalid data type: expected non-empty string."}}),
            false};
  }

  nlohmann::json body = {{"fname", path}};
  return CallApi("/project/jobs/delete_job", [this, &body](const std::string& endpoint) {
    return api_client_->Post(endpoint, "", body);
  });
}

}  // namespace hdrcl
