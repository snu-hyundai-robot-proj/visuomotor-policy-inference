#include <iostream>

#include "hdr_client_driver/hdr_client_driver.h"

int main() {
  try {
    hdrcl::HdrDriver driver("192.168.1.150");

    double api_version = driver.GetApiVersion();
    std::cout << "API Version: " << api_version << std::endl;

    double system_version = driver.GetSysVersion();
    std::cout << "System [com] Version: " << system_version << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
