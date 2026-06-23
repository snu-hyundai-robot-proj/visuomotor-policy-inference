#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {
/**
 * @param path The full directory path to query.
 *
 * @return A pair of:
 * - JSON array containing file entries.
 * - `true` if the status code is 200 (OK), otherwise `false`.
 *
 * @brief Retrieve a list of files and folders in the specified path.
 *
 * @details
 * This method sends a GET request to `/file_manager/files` with the specified path
 * and returns the list of file entries.
 *
 * Example usage:
 * @code
 * auto [result, ok] = driver.get_files("project/jobs/0001.job");
 * @endcode
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/1-files
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetFiles(const std::string& path) const {
  if (path.empty()) {
    return {nlohmann::json::object({{"error", "Invalid data type: expected string."}}), false};
  }

  return CallApi(
      "/file_manager/files",
      [this](const std::string& endpoint, const std::string& params) {
        auto [posts, status_code] = api_client_->GetStr(endpoint, params);
        try {
          return std::make_pair(nlohmann::json::parse(posts), status_code);
        } catch (const nlohmann::json::parse_error&) {
          nlohmann::json result = {{"content", posts}, {"is_json", false}};
          return std::make_pair(result, status_code);
        }
      },
      "pathname=" + path);
}

/**
 * @param path Full path to the target file or directory.
 *
 * @return A pair of:
 * - JSON object with file info.
 * - `true` if successful.
 *
 * @brief Retrieve metadata of a file or directory.
 *
 * @details
 * Sends a GET request to `/file_manager/file_info` to obtain file information such as size,
 * timestamp, and type.
 *
 * Example usage:
 * @code
 * auto [info, ok] = driver.get_file_info("project/jobs/0001.job");
 * @endcode
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/2-file_info
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetFileInfo(const std::string& path) const {
  if (path.empty()) {
    return {nlohmann::json::object({{"error", "Invalid data type: expected string."}}), false};
  }

  std::string params = "pathname=" + path;
  return CallApi("/file_manager/file_info", [this, &params](const std::string& endpoint) {
    return api_client_->Get(endpoint, params);
  });
}

/**
 * @param path Directory path to query.
 * @param incl_file If true, include files in result.
 * @param incl_dir If true, include directories in result.
 *
 * @return A pair of:
 * - JSON array of names.
 * - `true` if request was successful.
 *
 * @brief Get a filtered list of file or directory names.
 *
 * @details
 * Calls the `/file_manager/file_list` endpoint and allows filtering to include only
 * files, only directories, or both.
 *
 * Example usage:
 * @code
 * auto [list, ok] = driver.get_file_list("project", true, false); // files only
 * @endcode
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/3-file_list
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetFileList(const std::string& path, bool include_file,
                                                       bool include_directory) const {
  if (path.empty()) {
    return {nlohmann::json::object({{"error", "Invalid data type: expected non-empty string."}}),
            false};
  }

  std::string params = "path=" + path +
                       "&incl_file=" + std::string(include_file ? "true" : "false") +
                       "&incl_dir=" + std::string(include_directory ? "true" : "false");

  return CallApi("/file_manager/file_list", [this, &params](const std::string& endpoint) {
    return api_client_->Get(endpoint, params);
  });
}

/**
 * @param path Full path to the file or directory.
 *
 * @return A pair of:
 * - JSON with existence flag (e.g., `{"exists": true}`).
 * - `true` if HTTP status 200, otherwise `false`.
 *
 * @brief Check if a file or directory exists.
 *
 * @details
 * This method checks the presence of the file or folder using the endpoint
 * `/file_manager/file_exist`.
 *
 * Example usage:
 * @code
 * auto [exists, ok] = driver.get_file_exist("project/jobs/0001.job");
 * @endcode
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/4-file_exist
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::GetFileExist(const std::string& path) const {
  if (path.empty()) {
    return {nlohmann::json::object({{"error", "Invalid data type: expected string."}}), false};
  }

  return CallApi("/file_manager/file_exist", [this, &path](const std::string& endpoint) {
    return api_client_->Get(endpoint, "pathname=" + path);
  });
}

/**
 * @param pathname_from Original file/directory path.
 * @param pathname_to Destination file/directory path.
 *
 * @return A pair of:
 * - JSON response with result status.
 * - `true` if successful.
 *
 * @brief Rename or move a file or directory.
 *
 * @details
 * Uses the `/file_manager/rename_file` endpoint to rename or move a file/folder
 * from one path to another.
 *
 * Example usage:
 * @code
 * auto [res, ok] = driver.post_rename_file("project/jobs/0001.job", "project/jobs/4321.job");
 * @endcode
 *
 * @see
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/2-post/1-rename_file
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostRenameFile(const std::string& pathname_from,
                                                          const std::string& pathname_to) const {
  if (pathname_from.empty() || pathname_to.empty()) {
    return {nlohmann::json::object({{"error", "Invalid data type: expected non-empty strings."}}),
            false};
  }

  nlohmann::json param = {{"pathname_from", pathname_from}, {"pathname_to", pathname_to}};

  return CallApi("/file_manager/rename_file", [this, &param](const std::string& endpoint) {
    return api_client_->Post(endpoint, "", param);
  });
}

/**
 * @param path Full directory path to create.
 *
 * @return A pair of:
 * - JSON response with status message.
 * - `true` if status code is 200.
 *
 * @brief Create a new directory.
 *
 * @details
 * Sends a POST request to `/file_manager/mkdir` to create the specified directory.
 *
 * Example usage:
 * @code
 * auto [res, ok] = driver.post_mkdir("project/jobs/special");
 * @endcode
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/2-post/2-mkdir
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostMkdir(const std::string& path) const {
  if (path.empty()) {
    return {nlohmann::json::object({{"error", "Invalid data type: expected string."}}), false};
  }

  return CallApi("/file_manager/mkdir", [this, &path](const std::string& endpoint) {
    return api_client_->Post(endpoint, "", nlohmann::json{{"path", path}});
  });
}

/**
 * @param target_file Full destination path on the controller.
 * @param source_file Path to the local file to upload.
 *
 * @return A pair of:
 * - JSON upload result.
 * - `true` if successful (HTTP 200).
 *
 * @brief Upload a file to the controller.
 *
 * @details
 * Sends the contents of `source_file` to the controller at `target_file` location
 * via the `/file_manager/files` endpoint.
 *
 * Example usage:
 * @code
 * auto [res, ok] = driver.post_files("/project/jobs/test.job", "./test.job");
 * @endcode
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/2-post/3-files
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostFiles(const std::string& target_file,
                                                     const std::string& source_file) const {
  if (target_file.empty() || source_file.empty()) {
    return {nlohmann::json{{"error", "Invalid data type: expected non-empty strings."}}, false};
  }

  if (!std::filesystem::exists(source_file)) {
    return {nlohmann::json{{"error", "Invalid path: source_file does not exist."}}, false};
  }

  std::ifstream file(source_file, std::ios::binary);
  if (!file) {
    return {nlohmann::json{{"error", "Failed to open source file."}}, false};
  }

  std::string file_content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());

  std::string endpoint = "/file_manager/files/" + target_file;

  return CallApi(endpoint, [this, &file_content](const std::string& ep) {
    return api_client_->PostFile(ep, "", file_content);
  });
}

/**
 * @param path Full path of the file or directory to delete.
 *
 * @return A pair:
 * - JSON result of the deletion.
 * - `true` if the response status code is 200.
 *
 * @brief Delete a file or directory on the controller.
 *
 * @details
 * Sends a DELETE request to `/file_manager/files/{pathname}` to remove the specified file or
 * folder.
 *
 * Example usage:
 * @code
 * auto [res, ok] = driver.post_delete_file("/project/jobs/test.job");
 * @endcode
 * This is used for cleanup, remote file management, or preparing for new uploads.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/3-delete/1-files
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostDeleteFile(const std::string& path) const {
  if (path.empty()) {
    return {nlohmann::json::object({{"error", "Invalid data type: expected string."}}), false};
  }

  std::string job_path = "/file_manager/files/" + path;

  return CallApi(job_path,
                 [this](const std::string& endpoint) { return api_client_->Delete(endpoint); });
}
}  // namespace hdrcl