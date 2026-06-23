#pragma once
#include <boost/asio.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

namespace network {

/**
 * @brief A network socket manager supporting TCP client/server and UDP communication.
 *
 * @details
 * This class provides a unified interface for network communication using Boost.Asio.
 * It supports three modes: TCP client, TCP server, and UDP communication with
 * automatic connection management and asynchronous I/O operations.
 *
 * @class SocketManager
 */
class SocketManager {
 public:
  /**
   * @param mode Communication mode ("tcp_client", "tcp_server", or "udp")
   * @param host Hostname or IP address for connection
   * @param port Port number for network communication
   *
   * @brief Constructor.
   *
   * @details
   * Initializes the socket manager with the specified mode, host, and port.
   * Automatically sets up the appropriate socket type based on the mode parameter.
   */
  SocketManager(const std::string& mode, const std::string& host, int port);

  /**
   * @brief Destructor.
   *
   * @details
   * Ensures proper cleanup of network resources and stops the I/O thread.
   */
  ~SocketManager();

  /**
   * @param data String data to send over the network
   *
   * @brief Send string data through the established connection.
   *
   * @details
   * Transmits the provided string data using the configured communication mode.
   * The method handles different socket types transparently.
   */
  void SendData(const std::string& data);

  /**
   * @param values Vector of double values to send
   *
   * @brief Send numeric data through the established connection.
   *
   * @details
   * Converts the vector of double values to an appropriate format and
   * transmits them over the network connection.
   */
  void SendData(const std::vector<double>& values);

  /**
   * @return Received data as string
   *
   * @brief Receive data from the network connection.
   *
   * @details
   * Blocks until data is received from the remote endpoint and returns
   * the received data as a string. The method handles different socket
   * types and protocols transparently.
   */
  std::string ReceiveData();

  /**
   * @return Received line as string, or empty string if no data available
   *
   * @brief Non-blocking attempt to receive a line of data.
   *
   * @details
   * Unlike ReceiveData(), this method returns immediately without blocking.
   * Returns empty string if no complete line is available in the buffer.
   */
  std::string TryReceiveLineNonBlocking();

 private:
  /**
   * @brief Set up TCP client socket connection.
   *
   * @details
   * Initializes and connects a TCP client socket to the specified host and port.
   */
  void SetupTcpClient();

  /**
   * @brief Set up TCP server socket for accepting connections.
   *
   * @details
   * Creates a TCP acceptor and waits for incoming client connections
   * on the specified port.
   */
  void SetupTcpServer();

  /**
   * @brief Set up UDP socket for datagram communication.
   *
   * @details
   * Initializes a UDP socket for sending and receiving datagrams
   * to/from the specified remote endpoint.
   */
  void SetupUdp();

  /**
   * @param mode Mode string to validate
   *
   * @return True if mode is valid, false otherwise
   *
   * @brief Validate the communication mode parameter.
   *
   * @details
   * Checks if the provided mode string is one of the supported
   * communication modes: "tcp_client", "tcp_server", or "udp".
   */
  bool IsValidMode(const std::string& mode) const;

  /**
   * @param mode Mode string to normalize
   *
   * @return Normalized mode string
   *
   * @brief Normalize the mode string to standard format.
   *
   * @details
   * Converts the input mode string to lowercase and standardizes
   * the format for consistent internal usage.
   */
  std::string NormalizeMode(const std::string& mode) const;

  std::string mode_;  ///< Communication mode (tcp_client, tcp_server, udp)
  std::string host_;  ///< Hostname or IP address
  int port_;          ///< Port number for communication

  boost::asio::io_context io_context_;      ///< Boost.Asio I/O context
  std::unique_ptr<std::thread> io_thread_;  ///< Background I/O thread

  std::unique_ptr<boost::asio::ip::tcp::socket> tcp_socket_;      ///< TCP socket
  std::unique_ptr<boost::asio::ip::tcp::acceptor> tcp_acceptor_;  ///< TCP acceptor
  std::unique_ptr<boost::asio::ip::udp::socket> udp_socket_;      ///< UDP socket
  boost::asio::ip::udp::endpoint udp_remote_endpoint_;            ///< UDP remote endpoint

  boost::asio::streambuf tcp_buf_;  ///< TCP stream buffer for line-based reading
  std::vector<char> buffer_;        ///< Internal buffer for data operations
};
}  // namespace network
