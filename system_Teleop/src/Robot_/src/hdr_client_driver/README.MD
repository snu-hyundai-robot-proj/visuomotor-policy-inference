# HD Hyundai Robotics Client Driver Library

#### Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)

---

## 1. Overview

The HDR Client Driver provides a comprehensive C++ library for communicating with HD Hyundai Robotics' robot controllers via both HTTP (Open API) and socket Stream interfaces. This library abstracts communication layers and offers a clean, object-oriented interface for controlling and monitoring robots, managing files, executing real-time commands, and integrating with ROS2.

> ❗ **Note:** The socket Stream interface provides real-time joint state monitoring and trajectory command streaming at up to 500Hz. Official firmware release is scheduled for **Q2 2026**.

> ❗ **Important:** All REST API-based communication requires the robot to be in **REMOTE mode**.
---

## 2. Package Structure

| Directory         | Description                                                                |
|-------------------|----------------------------------------------------------------------------|
| `include/`        | Header files for the HDR Client Driver library.                            |
| `src/`            | Source implementation of the client driver functions.                      |
| `src/functions/`  | API category implementations for different robot controller functionality. |
| `examples/`       | Example programs demonstrating how to use the driver API.                  |

---

## 3. Installation

#### Installing Non-ROS dependencies

- C++ 17 compiler
- libcurl (for HTTP communication)
- nlohmann/json (for JSON parsing)
- boost/asio (for Socket communication)

```bash
sudo apt update
sudo apt install -y libcurl4-openssl-dev nlohmann-json3-dev
```

#### Build the package
##### Create a ROS2 workspace
```bash
# Skip if you already have a workspace
mkdir -p ~/ros2_ws/src/
```

##### Clone the latest repositories
```bash
cd ~/ros2_ws/src/
git clone https://github.com/hyundai-robotics/hdr_client_driver
```

##### Create and enter build directory
```bash
cd hdr_client_driver
mkdir -p build && cd build
```

##### Configure the project
```bash
cmake ..
```

##### Build the shared library
```bash
make -j$(nproc)
```


##### (Optional) Install to system paths
```bash
sudo make install
```

---

## 4. Usage

> ❗ **Note** The HDRDriver class requires a valid robot IP address and API port to establish a connection with the robot controller.

#### Example

##### Example 1: Monitor Joint Positions, Velocities, and Efforts

```cpp
#include <iostream>
#include <iomanip>
#include "hdr_client_driver/hdr_client_driver.h"

int main() {
    try {
        // Initialize driver
        hdrcl::HdrDriver driver("192.168.1.150");

        // Get system version
        auto sys_ver = driver.GetSysVersion();
        std::cout << "System Version: " << sys_ver << std::endl;

        // Perform socket Stream handshake (required before polling)
        const std::string stream_ver = "1.0.0";
        if (!driver.DoHandshake(stream_ver, 5000)) {
            std::cerr << "Handshake failed!" << std::endl;
            return 1;
        }

        // Start polling at 100Hz
        if (!driver.StartPolling(100)) {
            std::cerr << "Failed to start polling!" << std::endl;
            return 1;
        }

        // Wait for initial data
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Get joint data (positions, velocities, and efforts)
        auto joint_data = driver.GetJointData();

        if (joint_data.IsValid()) {
            std::cout << "Joint Positions (rad): ";
            for (const auto& pos : joint_data.positions) {
                std::cout << std::fixed << std::setprecision(4) << pos << " ";
            }
            std::cout << "\n";

            std::cout << "Joint Velocities (rad/s): ";
            for (const auto& vel : joint_data.velocities) {
                std::cout << std::fixed << std::setprecision(4) << vel << " ";
            }
            std::cout << "\n";

            std::cout << "Joint Efforts (torque): ";
            for (const auto& eff : joint_data.efforts) {
                std::cout << std::fixed << std::setprecision(2) << eff << " ";
            }
            std::cout << "\n";
        }

        // Cleanup
        driver.StopPolling();

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
```

##### Example Set Robot Position

```cpp
#include <iostream>
#include <vector>
#include <cmath>
#include "hdr_client_driver/hdr_client_driver.h"

// Helper function: degrees to radians
inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

int main() {
    try {
        hdrcl::HdrDriver driver("192.168.1.150");

        // Perform handshake
        const std::string stream_ver = "1.0.0";
        driver.DoHandshake(stream_ver, 5000);

        // Start polling
        driver.StartPolling(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Get current position
        auto current = driver.GetJointData();
        std::cout << "Current position acquired" << std::endl;
        
        // Activate motor and clear stops
        driver.PostRobotMotorPower();
        driver.RelayExternalStopClear();
        driver.PostInitJointTrajectory();
        driver.SetJobProgram(100);
        
        // Define target position [0, 90, 0, 0, 0, 0] degrees
        std::vector<double> target = {
            deg2rad(0.0), deg2rad(90.0), deg2rad(0.0),
            deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)
        };
        
        // Send trajectory commands at 100Hz
        const int control_hz = 100;
        const double look_ahead_time = 0.05;  // 50ms buffer
        
        for (int i = 0; i < 500; ++i) {  // 5 seconds of motion
            driver.SetRobotPosition(target, control_hz, look_ahead_time);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        std::cout << "Motion completed!" << std::endl;
        driver.StopPolling();
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```

#### API Categories

The HDR Client Driver supports the following API categories:
> Note: Most of the APIs below (especially motion and I/O) only work when the robot is in REMOTE mode.

##### Socket Stream APIs

| Function | Description |
|----------|-------------|
| `DoHandshake(stream_ver, timeout_ms)` | Performs socket Stream handshake with robot controller using semantic version string (e.g., "1.0.0"). Must be called before StartPolling(). |
| `StartPolling(hz)` | Starts background thread for real-time joint data streaming at specified frequency (1-500Hz). |
| `StopPolling()` | Stops the polling thread and closes socket Stream connection. |
| `GetJointData()` | Retrieves current joint positions, velocities, and efforts from the polling cache. Thread-safe. |
| `SetRobotPosition(position, hz, look_ahead_time)` | Sends trajectory point via socket Stream for real-time motion control. |
| `SetJobProgram(hz)` | Configures robot job program for online trajectory control at specified frequency. |
| `InitJointTrajectory()` | Initializes trajectory buffer. Should be called before sending trajectory commands. |

##### REST API Categories

| Category | Function | Description |
|----------|-----------|-------------|
| **Version** | `GetApiVersion` | Retrieves the HDR Open API version supported by the controller (e.g., 1.01) |
| | `GetSysVersion` | Returns the system version of the communication module ('com') from the controller (e.g., 60.29 from "60.29-02.dev") |
| **Project** | `GetProjectRgen` | Gets the current project execution state (0: Not running, 1: Running, 2: Paused) |
| | `GetProjectJobsInfo` | Retrieves metadata of all jobs registered in the project (name, path, modification status) |
| | `PostProjectReloadUpdateJobs` | Reloads and synchronizes jobs modified externally, updates in-memory job states |
| | `PostProjectDeleteJob` | Deletes a specified job file from the project path |
| **Control** | `GetControlOpCnd` | Retrieves the robot controller's execution condition configuration (playback mode, step back max speed, user coordinate number) |
| | `GetControlIosDio` | Reads the value of a specific digital I/O signal (supports types: "di", "dib", "diw", "dil", "dif", "do", "dob", "dow", "dol", "dof") |
| | `GetControlIosSio` | Gets the value of a Special I/O (SIO) signal (input types: "si", "sib", etc., output types: "so", "sob", etc.) |
| | `GetControlUcsNos` | Retrieves the list of available user coordinate system (UCS) numbers for motion programming |
| | `PostControlIosDio` | Sets the value of a digital output (DO) signal (type, block number, signal number, and value) |
| | `PutControlOpCnd` | Updates operation condition parameters (playback mode, maximum speed for reverse motion, user coordinate system) |
| **Robot** | `GetRobotMotorState` | Checks if the robot servo motor is powered on (ON/OFF), useful for confirming readiness for motion commands |
| | `GetRobotPoCur` | Gets the current robot pose (position & orientation) with various options (task index, coordinate system, etc.) |
| | `GetRobotCurTool` | Retrieves information about the currently selected tool (TCP configuration, weight, etc.) |
| | `GetRobotTools` | Gets a list of all registered tools in the system (TCP offsets, weights, etc.) |
| | `GetRobotToolsT` | Retrieves detailed information for a specific tool by tool number (0-31) |
| | `PostRobotMotorPower` | Turns the robot motor ON |
| | `PostRobotOperation` | Starts or stops the robot program execution |
| | `PostRobotToolNo` | Sets the active tool number to be used (0-31) |
| | `PostRobotCrdSys` | Specifies the coordinate system to use for motion and I/O (-1: default, 0: base, 1: tool, 2: user1, 3: user2) |
| | `PostRobotEmergencyStop` | Triggers an immediate emergency stop of all robot motion for safety response |
| | `PostRobotEmergencyStopTest` | Executes configurable emergency stop with options for immediate/deceleration/pause modes at specified step and position |
| | `GetJointTrajBuffAvail` | Retrieves the number of available slots in the joint trajectory buffer (max 2048), used to check space before sending new trajectory data. |
| | `PostInitJointTrajectory` | Initializes (clears) the joint trajectory buffer; must be called when the robot is stopped before sending new motion commands. |
| | `PostInsertJointTrajectoryPoints` | Inserts joint trajectory points (positions and timing) into the controller buffer for execution; supports continuous or stepwise motion. |
| **IO PLC** | `GetRelayValue` | Retrieves relay values from the Hi6 PLC using either "FB{index}.{relay_type}" format or simple format like "M", "S" |
| | `SetRelayValue` | Sets a specific relay value in the robot controller's internal PLC. Supports various data type suffixes |
| **File Manager** | `GetFiles` | Retrieves a list of files and folders in the specified path |
| | `GetFileInfo` | Gets metadata of a file or directory (size, timestamp, type) |
| | `GetFileList` | Retrieves a filtered list with options to include only files, only directories, or both |
| | `GetFileExist` | Checks if a specified file or directory exists |
| | `PostRenameFile` | Renames or moves a file or directory from one path to another |
| | `PostMkdir` | Creates a new directory at the specified path |
| | `PostFiles` | Uploads a local file to a specified location on the controller |
| | `PostDeleteFile` | Deletes a file or directory on the controller |
| **Task** | `PostReset` | Resets all tasks to initial state using RCode 0 execution |
| | `PostAssignVar` |  Assigns a variable to the task using an expression or JSON value (supports local/global scope and persistence) |
| | `PostReleaseWait` | Releases task[0] from WAIT state, resuming paused tasks |
| | `PostSetCurPcIdx` | Manually sets the program counter (PC) index for task[0] (useful for debugging or jumping to specific logic) |
| | `PostSolveExpr` | Evaluates an expression within the task scope (supports math, logic, and variable access) |
| | `PostExecuteMove` | Executes a direct move command in a robot task (L, P, SP, etc.) |
| **Console** | `ExecuteCommand` | Executes console commands via the HDR Open API (supports fixed commands like rl.stop, rl.reinit and motion commands) |
| **etc** | `GetDateTime` | Gets the current system date and time from the robot controller (year, month, day, hour, minute, second) |
| | `PutDateTime` | Sets the robot controller's system date and time (with input validation) |
| | `GetLogManager` | Retrieves logs from the controller with filtering options (number of items, categories E,W,N,S,O,I,P,H,C,M, ID range, timestamp range) |

Most of the above functions return values in the form of `std::pair<nlohmann::json, bool>`

---

## 5. Troubleshooting

Please refer to the [HD Hyundai Robotics Open API Documentation](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/) for more details on API functionality and how to troubleshoot common issues.

## Maintainers and License

- **Maintainer**: HD Hyundai Robotics R&D Team
- **License**: This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE)
- **Contact**: [kwon.hyojun@hd.com]
