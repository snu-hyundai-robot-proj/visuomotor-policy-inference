#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "hdr_client_driver/hdr_client_driver.h"

/**
 * @brief Example: Move robot to target position with smooth trajectory
 *
 * This example demonstrates:
 * 1. Reading current joint positions
 * 2. Moving to target position [0, 90, 0, 0, 0, 0] degrees
 * 3. Using linear interpolation for smooth motion
 */

// Convert degrees to radians
inline double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

// Convert radians to degrees
inline double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

// Linear interpolation
inline double lerp(double start, double end, double t) {
  return start + (end - start) * t;
}

int main() {
  try {
    // Initialize driver
    hdrcl::HdrDriver driver("192.168.1.150");

    std::cout << "===============================================" << std::endl;
    std::cout << "HDR Robot - Move to Target Position" << std::endl;
    std::cout << "===============================================" << std::endl;

    // Get system version
    auto sys_ver = driver.GetSysVersion();
    std::cout << "System Version: " << sys_ver << std::endl;

    // Perform handshake
    std::cout << "Performing handshake..." << std::endl;
    const std::string stream_ver = "1.0.0";
    if (!driver.DoHandshake(stream_ver, 5000)) {
      std::cerr << "[ERROR] Handshake failed!" << std::endl;
      return 1;
    }
    std::cout << "[SUCCESS] Handshake completed" << std::endl;

    // Start polling
    constexpr int control_hz = 100;
    std::cout << "Starting polling at " << control_hz << " Hz..." << std::endl;
    if (!driver.StartPolling(control_hz)) {
      std::cerr << "[ERROR] Failed to start polling!" << std::endl;
      return 1;
    }
    std::cout << "[SUCCESS] Polling started" << std::endl;

    // Wait for initial data
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Get current position
    auto current_data = driver.GetJointData();
    if (!current_data.IsValid() || current_data.Size() < 6) {
      std::cerr << "[ERROR] Failed to get current joint data!" << std::endl;
      return 1;
    }

    std::cout << "\nCurrent Position (degrees):" << std::endl;
    for (size_t i = 0; i < current_data.positions.size(); ++i) {
      std::cout << "  J" << (i + 1) << ": " << std::fixed << std::setprecision(2)
                << rad2deg(current_data.positions[i]) << "°" << std::endl;
    }

    // Define target position in degrees
    std::vector<double> target_degrees = {0.0, 90.0, 0.0, 0.0, 0.0, 0.0};

    // Convert to radians
    std::vector<double> target_position(6);
    for (size_t i = 0; i < 6; ++i) {
      target_position[i] = deg2rad(target_degrees[i]);
    }

    std::cout << "\nTarget Position (degrees):" << std::endl;
    for (size_t i = 0; i < target_position.size(); ++i) {
      std::cout << "  J" << (i + 1) << ": " << std::fixed << std::setprecision(2)
                << target_degrees[i] << "°" << std::endl;
    }

    // Calculate maximum angular distance for any joint
    double max_distance = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      double distance = std::abs(target_position[i] - current_data.positions[i]);
      max_distance = std::max(max_distance, distance);
    }

    std::cout << "\nMaximum joint movement: " << std::fixed << std::setprecision(2)
              << rad2deg(max_distance) << "°" << std::endl;

    // Activate motor power
    std::cout << "\nActivating motor power..." << std::endl;
    auto [motor_result, motor_ok] = driver.PostRobotMotorPower();
    if (!motor_ok) {
      std::cerr << "[ERROR] Failed to activate motor!" << std::endl;
      return 1;
    }
    std::cout << "[SUCCESS] Motor activated" << std::endl;

    // Clear external stop
    std::cout << "Clearing external stop..." << std::endl;
    auto [relay_result, relay_ok] = driver.RelayExternalStopClear();
    if (!relay_ok) {
      std::cerr << "[ERROR] Failed to clear external stop!" << std::endl;
      return 1;
    }
    std::cout << "[SUCCESS] External stop cleared" << std::endl;

    // Initialize trajectory buffer
    std::cout << "Initializing joint trajectory..." << std::endl;
    auto [traj_result, traj_ok] = driver.PostInitJointTrajectory();
    if (!traj_ok) {
      std::cerr << "[ERROR] Failed to initialize trajectory!" << std::endl;
      return 1;
    }
    std::cout << "[SUCCESS] Trajectory initialized" << std::endl;

    // Setup job program
    std::cout << "Setting up job program..." << std::endl;
    if (!driver.SetJobProgram(control_hz)) {
      std::cerr << "[ERROR] Failed to set job program!" << std::endl;
      return 1;
    }
    std::cout << "[SUCCESS] Job program configured" << std::endl;

    // Motion parameters
    constexpr double motion_speed = 0.1;  // 0.2 rad/s (~11.5 deg/s) - SLOW AND SAFE
    const double motion_duration = max_distance / motion_speed;  // Time to complete motion
    constexpr int buffer_size = 10;
    const double look_ahead_time = (1.0 / control_hz) * buffer_size;
    const auto control_interval = std::chrono::milliseconds(1000 / control_hz);

    std::cout << "\n===============================================" << std::endl;
    std::cout << "Starting motion to target..." << std::endl;
    std::cout << "  Motion speed: " << motion_speed << " rad/s (~" << rad2deg(motion_speed)
              << " deg/s)" << std::endl;
    std::cout << "  Estimated duration: " << std::fixed << std::setprecision(1) << motion_duration
              << " seconds" << std::endl;
    std::cout << "  Control rate: " << control_hz << " Hz" << std::endl;
    std::cout << "  Look-ahead time: " << look_ahead_time << " seconds" << std::endl;
    std::cout << "===============================================" << std::endl;

    // Main control loop
    using Clock = std::chrono::steady_clock;
    auto start_time = Clock::now();
    int command_count = 0;
    bool motion_complete = false;

    while (!motion_complete) {
      auto loop_start = Clock::now();
      auto elapsed = std::chrono::duration<double>(loop_start - start_time).count();

      // Calculate interpolation parameter (0.0 to 1.0)
      double t = std::min(elapsed / motion_duration, 1.0);

      // Interpolate from current to target position
      std::vector<double> command_position(6);
      for (size_t i = 0; i < 6; ++i) {
        command_position[i] = lerp(current_data.positions[i], target_position[i], t);
      }

      // Send position command
      if (!driver.SetRobotPosition(command_position, control_hz, look_ahead_time)) {
        std::cerr << "\n[ERROR] Failed to send position command!" << std::endl;
        break;
      }

      command_count++;

      // Print status every 50 commands (0.5 seconds at 100Hz)
      if (command_count % 50 == 0) {
        std::cout << "Progress: " << std::fixed << std::setprecision(1) << (t * 100.0) << "% | "
                  << "Time: " << std::setprecision(2) << elapsed << "s | "
                  << "Commands: " << command_count << std::endl;

        // Show current joint positions
        std::cout << "  Current (deg): ";
        for (size_t i = 0; i < command_position.size(); ++i) {
          std::cout << "J" << (i + 1) << ":" << std::setprecision(1) << rad2deg(command_position[i])
                    << "° ";
        }
        std::cout << std::endl;
      }

      // Check if motion is complete
      if (t >= 1.0) {
        motion_complete = true;
        std::cout << "\nMotion completed!" << std::endl;

        // Send final target position a few more times to ensure stability
        for (int i = 0; i < 10; ++i) {
          driver.SetRobotPosition(target_position, control_hz, look_ahead_time);
          std::this_thread::sleep_for(control_interval);
        }
      }

      // Maintain control rate
      auto loop_end = Clock::now();
      auto loop_duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);

      if (control_interval > loop_duration) {
        std::this_thread::sleep_for(control_interval - loop_duration);
      }
    }

    // Verify final position
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto final_data = driver.GetJointData();

    std::cout << "\n===============================================" << std::endl;
    std::cout << "Final Position (degrees):" << std::endl;
    for (size_t i = 0; i < final_data.positions.size(); ++i) {
      double error = std::abs(final_data.positions[i] - target_position[i]);
      std::cout << "  J" << (i + 1) << ": " << std::fixed << std::setprecision(2)
                << rad2deg(final_data.positions[i]) << "° "
                << "(error: " << rad2deg(error) << "°)" << std::endl;
    }
    std::cout << "===============================================" << std::endl;
    std::cout << "Total commands sent: " << command_count << std::endl;
    std::cout << "Motion control example completed successfully!" << std::endl;
    std::cout << "===============================================" << std::endl;

    // Cleanup
    driver.StopPolling();

  } catch (const std::exception& e) {
    std::cerr << "[EXCEPTION] " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
