#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "hdr_client_driver/hdr_client_driver.h"

int main() {
  try {
    // Initialize driver with robot IP
    hdrcl::HdrDriver driver("192.168.1.150");

    std::cout << "===============================================" << std::endl;
    std::cout << "HDR Robot Position Monitoring Example" << std::endl;
    std::cout << "===============================================" << std::endl;

    // Get robot firmware versions
    auto api_ver = driver.GetApiVersion();
    auto sys_ver = driver.GetSysVersion();
    std::cout << "API Version: " << api_ver << std::endl;
    std::cout << "System Version: " << sys_ver << std::endl;
    std::cout << "===============================================" << std::endl;

    // Perform socket Stream handshake
    std::cout << "Performing handshake..." << std::endl;
    const std::string stream_ver = "1.0.0";
    if (!driver.DoHandshake(stream_ver, 5000)) {
      std::cerr << "[ERROR] Handshake failed!" << std::endl;
      return 1;
    }
    std::cout << "[SUCCESS] Handshake completed" << std::endl;
    std::cout << "===============================================" << std::endl;

    // Start polling at 100Hz
    constexpr int poll_hz = 100;
    std::cout << "Starting polling at " << poll_hz << " Hz..." << std::endl;

    if (!driver.StartPolling(poll_hz)) {
      std::cerr << "[ERROR] Failed to start polling!" << std::endl;
      return 1;
    }
    std::cout << "[SUCCESS] Polling started" << std::endl;
    std::cout << "===============================================" << std::endl;
    std::cout << "Press Ctrl+C to stop monitoring" << std::endl;
    std::cout << "===============================================" << std::endl;

    // Wait a bit for initial data
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Main monitoring loop
    const auto poll_interval = std::chrono::milliseconds(1000 / poll_hz);
    using Clock = std::chrono::steady_clock;
    auto prev_time = Clock::now();
    int frame_count = 0;

    while (true) {
      auto loop_start = Clock::now();

      // Get joint data (positions, velocities, and efforts)
      auto joint_data = driver.GetJointData();

      auto loop_end = Clock::now();
      auto interval =
          std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - prev_time).count();
      prev_time = loop_end;

      // Print data every 10 frames (10Hz display rate)
      if (frame_count % 10 == 0) {
        auto timestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(loop_end.time_since_epoch())
                .count();

        std::cout << "\n[Frame " << frame_count << " | Time: " << timestamp
                  << " ms | Δt: " << interval << " ms]" << std::endl;

        if (joint_data.IsValid()) {
          std::cout << "  Joints: " << joint_data.Size() << std::endl;

          std::cout << "  Position (rad): ";
          for (size_t i = 0; i < joint_data.positions.size(); ++i) {
            std::cout << "J" << (i + 1) << ":" << std::fixed << std::setprecision(4)
                      << joint_data.positions[i] << " ";
          }
          std::cout << std::endl;

          std::cout << "  Velocity (rad/s): ";
          for (size_t i = 0; i < joint_data.velocities.size(); ++i) {
            std::cout << "J" << (i + 1) << ":" << std::fixed << std::setprecision(4)
                      << joint_data.velocities[i] << " ";
          }
          std::cout << std::endl;

          std::cout << "  Effort (torque): ";
          for (size_t i = 0; i < joint_data.efforts.size(); ++i) {
            std::cout << "J" << (i + 1) << ":" << std::fixed << std::setprecision(2)
                      << joint_data.efforts[i] << " ";
          }
          std::cout << std::endl;
        } else {
          std::cout << "  [WARN] Joint data not yet available" << std::endl;
        }
      }

      frame_count++;

      // Maintain polling rate
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
      if (poll_interval > elapsed) {
        std::this_thread::sleep_for(poll_interval - elapsed);
      }
    }

  } catch (const std::exception& e) {
    std::cerr << "[EXCEPTION] " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
