#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {

/**
 * @param pno Project number
 * @param sno Section number
 * @param fno File line number
 * @param ext_sel Extended selector (optional)
 *
 * @return A pair:
 * - JSON response
 * - true if HTTP 200 (success), false otherwise
 *
 * @brief Set the current program counter to a specific line in the program.
 *
 * @details
 * Allows manual setting of the PC (Program Counter) for task[0] by specifying the
 * project number, section number, file line number, and an optional extended selector.
 * This is commonly used for jumping to a specific execution point.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/1-cur_prog_cnt
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostCurProgCnt(int pno, int sno, int fno,
                                                          int ext_sel) const {
  nlohmann::json param = {{"pno", pno}, {"sno", sno}, {"fno", fno}, {"ext_sel", ext_sel}};
  return CallApi("/project/context/tasks[0]/cur_prog_cnt",
                 [this, &param](const std::string& endpoint) {
                   return api_client_->Post(endpoint, "", param);
                 });
}

/**
 * @return A pair:
 * - JSON response
 * - true if successful (HTTP 200)
 *
 * @brief Reset all tasks to their initial state.
 *
 * @details
 * Sends a reset command to all tasks in the context. This stops execution and resets internal
 * states.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/2-reset
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostReset() const {
  nlohmann::json body = {{"code", 0}};
  return CallApi("/project/service/r_code/execute", [this, &body](const std::string& endpoint) {
    return api_client_->Post(endpoint, "", body);
  });
}
/**
 * @param name Variable name
 * @param scope Scope: "local", "global", or ""
 * @param expr Expression or JSON string
 * @param save Whether to persist the value across reboots
 *
 * @return A pair:
 * - JSON result
 * - true if HTTP 200 (success)
 *
 * @brief Assign a variable to the task using an expression or a JSON value.
 *
 * @details
 * If `expr` is a valid JSON string, it will be assigned via `/assign_var_json`, otherwise via
 * `/assign_var_expr`. Scope defines the visibility of the variable (`local`, `global`, or empty for
 * default).
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/3-assign_var_expr
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/4-assign_var_json
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostAssignVar(const std::string& name,
                                                         const std::string& scope,
                                                         const std::string& expr, bool save) const {
  std::set<std::string> allowed_types = {"local", "global", ""};
  if (!allowed_types.count(scope)) {
    return {nlohmann::json{{"error", "Invalid request type"}}, false};
  }

  try {
    bool is_json = false;
    try {
      auto parsed = nlohmann::json::parse(expr);
      is_json = true;
    } catch (...) {
      // Not valid JSON, will use expr endpoint
    }

    nlohmann::json param = {
        {"name", name}, {"scope", scope}, {is_json ? "json" : "expr", expr}, {"save", save}};

    std::string endpoint = is_json ? "/project/context/tasks[0]/assign_var_json"
                                   : "/project/context/tasks[0]/assign_var_expr";

    return CallApi(endpoint, [this, &param](const std::string& endpoint) {
      return api_client_->Post(endpoint, "", param);
    });
  } catch (const std::exception& e) {
    return {nlohmann::json{{"error", e.what()}}, false};
  }
}

/**
 * @return A pair:
 * - JSON result
 * - true if released (HTTP 200)
 *
 * @brief Release task[0] from a WAIT state.
 *
 * @details
 * Tasks that are in WAIT state can be resumed using this call.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/5-release_wait
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostReleaseWait() const {
  return CallApi("/project/context/tasks[0]/release_wait", [this](const std::string& endpoint) {
    return api_client_->Post(endpoint, "", nlohmann::json::object());
  });
}
/**
 * @param idx PC index to set
 *
 * @return A pair:
 * - JSON result
 * - true if successful
 *
 * @brief Manually set the program counter (PC) index for task[0].
 *
 * @details
 * Changes the current PC to the specified index. Useful for debugging or jumping logic.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/6-set_cur_pc_idx
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostSetCurPcIdx(int idx) const {
  nlohmann::json param = {{"idx", idx}};
  return CallApi("/project/context/tasks[0]/set_cur_pc_idx",
                 [this, &param](const std::string& endpoint) {
                   return api_client_->Post(endpoint, "", param);
                 });
}

/**
 * @param scope "local", "global", or ""
 * @param expr Expression to evaluate (e.g. "3+4*var1")
 *
 * @return A pair:
 * - JSON result with evaluated value
 * - true if HTTP 200
 *
 * @brief Evaluate an expression within the task scope.
 *
 * @details
 * Supports math, logic, and variable access within the task's context.
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/7-solve_expr
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostSolveExpr(const std::string& scope,
                                                         const std::string& expr) const {
  std::set<std::string> allowed_scopes = {"local", "global", ""};
  if (!allowed_scopes.count(scope)) {
    return {nlohmann::json{{"error", "Invalid scope"}}, false};
  }

  if (expr.empty()) {
    return {nlohmann::json{{"error", "Expression cannot be empty"}}, false};
  }

  nlohmann::json param = {{"expr", expr}, {"scope", scope}};
  return CallApi("/project/context/tasks[0]/solve_expr",
                 [this, &param](const std::string& endpoint) {
                   return api_client_->Post(endpoint, "", param);
                 });
}

/**
 * @param stmt Full move statement string
 * @param task_no Task index (e.g. 0 for default)
 *
 * @return A pair:
 * - JSON execution result
 * - true if HTTP 200
 *
 * @brief Execute a move command in a robot task.
 *
 * @details
 * Executes a direct move statement such as linear (L), point-to-point (P), or spline (SP).
 * The move statement must begin with `"move "` and follow valid syntax.
 * Example: `move SP,spd=1sec,accu=0,tool=1 [0, 90, 0, 0, 0, 0]`
 *
 * @see https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/8-execute_move
 *
 */
std::pair<nlohmann::json, bool> HdrDriver::PostExecuteMove(const std::string& stmt,
                                                           int task_no) const {
  const std::string example = "Example: move SP,spd=1sec,accu=0,tool=1 [0, 90, 0, 0, 0, 0]";

  if (stmt.substr(0, 5) != "move ") {
    return {nlohmann::json{{"error", "Invalid command start. " + example}}, false};
  }

  size_t comma_pos = stmt.find(',');
  if (comma_pos == std::string::npos) {
    return {nlohmann::json{{"error", "Missing comma. " + example}}, false};
  }

  std::string move_type = stmt.substr(5, comma_pos - 5);
  std::vector<std::string> valid = {"P", "L", "C", "SP", "SL", "SC"};
  if (std::find(valid.begin(), valid.end(), move_type) == valid.end()) {
    return {nlohmann::json{{"error", "Invalid move type: " + move_type + ". " + example}}, false};
  }

  try {
    std::string path = "/project/context/tasks[" + std::to_string(task_no) + "]/execute_move";
    nlohmann::json body = {{"stmt", stmt}};
    return CallApi(path, [this, &body](const std::string& endpoint) {
      return api_client_->Post(endpoint, "", body);
    });
  } catch (const std::exception& e) {
    return {nlohmann::json{{"error", e.what()}}, false};
  }
}

}  // namespace hdrcl
