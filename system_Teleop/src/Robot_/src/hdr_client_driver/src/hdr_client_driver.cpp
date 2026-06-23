#include "hdr_client_driver/hdr_client_driver.h"

namespace hdrcl {
/**
 * @param robot_ip IP address of the robot controller (e.g., "192.168.1.150")
 *
 * @brief Initializes the driver with HTTP OpenAPI client only.
 *
 * @throws std::invalid_argument if input parameters are invalid
 * @throws hdrcl::util::ApiException if API client initialization fails
 * @throws std::runtime_error for other initialization failures
 *
 * @details
 * Sets up HTTP client for robot communication via OpenAPI.
 * This constructor is for scenarios where only API communication is needed,
 * without socket-based real-time communication.
 * Use the full constructor if you need both API and socket communication.
 */
HdrDriver::HdrDriver(const std::string& robot_ip) : robot_ip_(robot_ip) {
  // Validate input parameters
  hdrcl::util::ValidateIpAddress(robot_ip_);

  // Create OpenAPI URL (validation is handled inside MakeOpenApiUrl)
  std::string open_api_url = hdrcl::util::MakeOpenApiUrl(robot_ip_, openapi_port_);

  // Initialize API client with specific error handling
  try {
    api_client_ = std::make_shared<request::RequestLib>(open_api_url);
  } catch (const std::invalid_argument& e) {
    throw hdrcl::util::ApiException("Invalid API configuration: " + std::string(e.what()));
  } catch (const std::exception& e) {
    throw hdrcl::util::ApiException("Failed to initialize API client: " + std::string(e.what()));
  }
  // Initialize socket Stream manager
  try {
    stream_socket_ =
        std::make_shared<network::SocketManager>("TCP_CLIENT", robot_ip_, stream_port_);
  } catch (const std::invalid_argument& e) {
    throw std::invalid_argument("Invalid socket configuration: " + std::string(e.what()));
  } catch (const boost::system::system_error& e) {
    throw util::NetworkException("Connection failed to " + robot_ip_ + ":" +
                                 std::to_string(stream_port_) + " - " + e.what());
  } catch (const std::exception& e) {
    throw util::NetworkException("Socket initialization failed: " + std::string(e.what()));
  }

  // Log successful initialization
  std::cout << "[HDR Driver] API-only mode initialized - IP: " << robot_ip_
            << ", API port: " << openapi_port_ << std::endl;
}

/**
 * @brief Destructor - ensures all resources are properly cleaned up.
 *
 * @details
 * Performs safe shutdown of all threads and sockets:
 * - Stops polling and sending threads gracefully
 * - Releases socket Stream (destructor handles cleanup)
 * - Cleans up any pending commands
 *
 * All operations are exception-safe to prevent termination during cleanup.
 */
HdrDriver::~HdrDriver() {
  try {
    std::cout << "[HDR Driver] Shutting down driver..." << std::endl;

    // Stop threads first (this also clears queues and sends STOP command)
    StopPolling();  // This internally calls StopSending()

    // Reset socket connections (destructors will handle cleanup)
    if (stream_socket_) {
      std::cout << "[HDR Driver] Releasing socket Stream..." << std::endl;
      stream_socket_.reset();  // Explicitly trigger SocketManager destructor
      std::cout << "[HDR Driver] socket Stream released successfully" << std::endl;
    }

    std::cout << "[HDR Driver] Driver shutdown complete" << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] Exception during HdrDriver destruction: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "[ERROR] Unknown exception during HdrDriver destruction" << std::endl;
  }
}
}  // namespace hdrcl
