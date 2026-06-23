#include "hdr_client_driver/socket_manager.h"

#include <algorithm>
#include <boost/asio/ip/host_name.hpp>
#include <iostream>
#include <stdexcept>

namespace network {

/**
 * @param mode Communication mode string to initialize socket
 * @param host Hostname or IP address for connection
 * @param port Port number for network communication
 *
 * @brief Constructor that initializes socket manager with specified parameters.
 *
 * @throws std::invalid_argument if the provided mode is not supported
 * @throws std::exception if socket initialization fails
 *
 * @details
 * Validates the communication mode, normalizes it to uppercase format,
 * and sets up the appropriate socket type. Starts a background I/O thread
 * to handle asynchronous operations.
 */
SocketManager::SocketManager(const std::string& mode, const std::string& host, int port)
    : mode_(NormalizeMode(mode)), host_(host), port_(port), buffer_(1024) {
  if (!IsValidMode(mode_)) {
    throw std::invalid_argument("Invalid socket mode: " + mode +
                                ". Valid modes: TCP_CLIENT, TCP_SERVER, UDP");
  }

  try {
    if (mode_ == "TCP_CLIENT") {
      SetupTcpClient();
    } else if (mode_ == "TCP_SERVER") {
      SetupTcpServer();
    } else if (mode_ == "UDP") {
      SetupUdp();
    }

    // Start I/O context on separate thread for asynchronous operations
    io_thread_ = std::make_unique<std::thread>([this]() {
      try {
        io_context_.run();
      } catch (const std::exception& e) {
        std::cerr << "IO context error: " << e.what() << std::endl;
      }
    });

  } catch (const std::exception& e) {
    std::cerr << "SocketManager init failed: " << e.what() << std::endl;
    throw;
  }
}

/**
 * @brief Destructor that ensures proper cleanup of network resources.
 *
 * @details
 * Stops the I/O context and joins the background thread to prevent
 * resource leaks and ensure graceful shutdown.
 */
SocketManager::~SocketManager() {
  io_context_.stop();
  if (io_thread_ && io_thread_->joinable()) {
    io_thread_->join();
  }
}

/**
 * @param mode Mode string to validate against supported modes
 *
 * @return True if mode is valid, false otherwise
 *
 * @brief Validate if the provided mode string is supported.
 *
 * @details
 * Checks if the normalized mode string matches one of the supported
 * communication modes: TCP_CLIENT, TCP_SERVER, or UDP.
 */
bool SocketManager::IsValidMode(const std::string& mode) const {
  return mode == "TCP_CLIENT" || mode == "TCP_SERVER" || mode == "UDP";
}

/**
 * @param mode Original mode string to normalize
 *
 * @return Normalized mode string in uppercase format
 *
 * @brief Normalize mode string to standard uppercase format with alias handling.
 *
 * @details
 * Converts the input mode to uppercase and handles common aliases.
 * For example, "TCP" is automatically converted to "TCP_CLIENT" as default.
 */
std::string SocketManager::NormalizeMode(const std::string& mode) const {
  std::string normalized = mode;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), ::toupper);

  // Handle common aliases
  if (normalized == "TCP") {
    return "TCP_CLIENT";  // Default TCP mode to client
  }

  return normalized;
}

/**
 * @brief Set up TCP client socket and establish connection to remote host.
 *
 * @throws boost::system::system_error if connection fails
 *
 * @details
 * Creates a TCP socket, resolves the hostname to endpoints, and establishes
 * a connection to the specified host and port. Prints connection status
 * to standard output upon successful connection.
 */
void SocketManager::SetupTcpClient() {
  tcp_socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
  boost::asio::ip::tcp::resolver resolver(io_context_);
  auto endpoints = resolver.resolve(host_, std::to_string(port_));
  boost::asio::connect(*tcp_socket_, endpoints);

  tcp_socket_->set_option(boost::asio::ip::tcp::no_delay(true));

  tcp_socket_->set_option(boost::asio::socket_base::keep_alive(true));

  tcp_socket_->set_option(boost::asio::socket_base::receive_buffer_size(1 << 20));
  tcp_socket_->set_option(boost::asio::socket_base::send_buffer_size(1 << 20));

  std::ios::sync_with_stdio(false);

  std::cout << "[TCP CLIENT] Connected to " << host_ << ":" << port_ << "\n";
}

/**
 * @brief Set up TCP server socket and wait for incoming client connections.
 *
 * @throws boost::system::system_error if server setup or accept fails
 *
 * @details
 * Creates a TCP acceptor bound to the specified port, then waits for and
 * accepts the first incoming client connection. This is a blocking operation
 * that will wait until a client connects.
 */
void SocketManager::SetupTcpServer() {
  tcp_acceptor_ = std::make_unique<boost::asio::ip::tcp::acceptor>(
      io_context_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port_));

  tcp_socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
  tcp_acceptor_->accept(*tcp_socket_);
  std::cout << "[TCP SERVER] Client connected on port " << port_ << "\n";
}

/**
 * @brief Set up UDP socket for datagram communication.
 *
 * @throws boost::system::system_error if UDP socket creation fails
 *
 * @details
 * Creates a UDP socket bound to the specified port and sets up the remote
 * endpoint for sending datagrams. The socket is ready for both sending
 * and receiving UDP packets.
 */
void SocketManager::SetupUdp() {
  udp_socket_ = std::make_unique<boost::asio::ip::udp::socket>(
      io_context_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port_));

  udp_socket_->set_option(boost::asio::socket_base::receive_buffer_size(1 << 20));
  udp_socket_->set_option(boost::asio::socket_base::send_buffer_size(1 << 20));

  udp_socket_->set_option(boost::asio::socket_base::reuse_address(true));

  udp_remote_endpoint_ =
      boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(host_), port_);
  {
    int tos = 0x10;  // LOWDELAY
    ::setsockopt(udp_socket_->native_handle(), IPPROTO_IP, IP_TOS, &tos, sizeof(tos));
  }
  std::cout << "[UDP] Socket ready on port " << port_ << "\n";
}

/**
 * @param data String data to transmit over the network
 *
 * @brief Send string data through the established network connection.
 *
 * @throws std::runtime_error if socket is not initialized or send operation fails
 *
 * @details
 * Transmits the provided string data using the appropriate protocol (TCP or UDP)
 * based on the current socket mode. For TCP, uses synchronous write operation.
 * For UDP, sends datagram to the configured remote endpoint.
 */
void SocketManager::SendData(const std::string& data) {
  if (!tcp_socket_ && !udp_socket_) {
    throw std::runtime_error("Socket not initialized. Cannot send data.");
  }

  try {
    if (mode_ == "TCP_CLIENT" || mode_ == "TCP_SERVER") {
      if (!tcp_socket_) {
        throw std::runtime_error("TCP socket not initialized");
      }
      boost::asio::write(*tcp_socket_, boost::asio::buffer(data));
    } else if (mode_ == "UDP") {
      if (!udp_socket_) {
        throw std::runtime_error("UDP socket not initialized");
      }
      udp_socket_->send_to(boost::asio::buffer(data), udp_remote_endpoint_);
    }
  } catch (const std::exception& e) {
    std::cerr << "Send error: " << e.what() << std::endl;
    throw;
  }
}

/**
 * @param values Vector of double values to send as JSON
 *
 * @brief Send numeric data as JSON through the network connection.
 *
 * @details
 * Converts the vector of double values to JSON format using nlohmann::json
 * library and sends the serialized JSON string through the network connection.
 * This provides a convenient way to transmit structured numeric data.
 */
void SocketManager::SendData(const std::vector<double>& values) {
  nlohmann::json j = values;
  SendData(j.dump());
}

/**
 * @return Received data as string, or empty string on error
 *
 * @brief Receive data from the network connection.
 *
 * @throws std::runtime_error if socket is not initialized
 *
 * @details
 * Blocks until data is received from the remote endpoint. For TCP connections,
 * uses read_some to receive available data. For UDP, uses receive_from to
 * get datagrams from any sender. Returns the received data as a string,
 * or empty string if an error occurs during reception.
 */
std::string SocketManager::ReceiveData() {
  if (!tcp_socket_ && !udp_socket_) {
    throw std::runtime_error("Socket not initialized. Cannot receive data.");
  }

  try {
    if (mode_ == "TCP_CLIENT" || mode_ == "TCP_SERVER") {
      if (!tcp_socket_)
        throw std::runtime_error("TCP socket not initialized");

      boost::system::error_code ec;
      boost::asio::read_until(*tcp_socket_, tcp_buf_, '\n', ec);
      if (ec && ec != boost::asio::error::eof) {
        std::cerr << "Receive error: " << ec.message() << "\n";
        return {};
      }

      std::istream is(&tcp_buf_);
      std::string line;
      std::getline(is, line);  // '\n' 제거
      if (!line.empty() && line.back() == '\r')
        line.pop_back();  // CRLF 처리
      return line;
    } else if (mode_ == "UDP") {
      if (!udp_socket_) {
        throw std::runtime_error("UDP socket not initialized");
      }
      boost::asio::ip::udp::endpoint sender_endpoint;
      std::size_t len = udp_socket_->receive_from(boost::asio::buffer(buffer_), sender_endpoint);
      return std::string(buffer_.data(), len);
    }
  } catch (const std::exception& e) {
    std::cerr << "Receive error: " << e.what() << "\n";
    return {};
  }
}

std::string SocketManager::TryReceiveLineNonBlocking() {
  if (!(mode_ == "TCP_CLIENT" || mode_ == "TCP_SERVER"))
    return {};

  if (!tcp_socket_)
    return {};

  try {
    // Check if we have a complete line in the buffer first
    auto seq = tcp_buf_.data();
    for (auto it = boost::asio::buffers_begin(seq); it != boost::asio::buffers_end(seq); ++it) {
      if (*it == '\n') {
        std::istream is(&tcp_buf_);
        std::string line;
        std::getline(is, line);
        if (!line.empty() && line.back() == '\r')
          line.pop_back();
        return line;
      }
    }
  } catch (...) {
    // Silently ignore errors in non-blocking mode
  }

  return {};
}

}  // namespace network
