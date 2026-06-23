#include <chrono>
#include <iostream>
#include <thread>

#include "hdr_client_driver/hdr_client_driver.h"

int main() {
  try {
    hdrcl::HdrDriver driver("192.168.1.150");

    std::pair<nlohmann::json, bool> log_response;

    log_response =
        driver.GetLogManager(5, "W", std::nullopt, std::nullopt, std::nullopt, std::nullopt);

    std::cout << "First request results:\n";
    std::cout << "Success: " << (log_response.second ? "true" : "false") << "\n";
    std::cout << "Data:\n" << log_response.first.dump(2) << "\n";

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "\nSecond request (matching Python parameters):\n";

    log_response = driver.GetLogManager(100, "E,W,P,O", 10, 20000, std::nullopt, std::nullopt);

    std::cout << "Success: " << (log_response.second ? "true" : "false") << "\n";
    std::cout << "Data:\n" << log_response.first.dump(2) << "\n";

  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
