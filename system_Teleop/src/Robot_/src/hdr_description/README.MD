# HD Hyundai Robotics Description

### Table of Contents
- [Overview](#overview)
- [Package Structure](#package-structure)
- [Installation](#installation)
- [Usage](#usage)

---

## 1. Overview
A collection of robot model descriptions, meshes, and visualization configurations for HD Hyundai Robotics (HDR) robots in ROS2.



## 2. Package Structure
This package contains the following packages:
| Package             | Description                                                                                  |
|---------------------|----------------------------------------------------------------------------------------------|
| `launch/`           | Contains launch file to visualize HDR robots in RViz.                               |
| `rviz/`             | RViz configuration for display_robot.launch.py. |
| `urdf/`             | Contains the robot description files used to define the physical, visual, and control properties of HD Hyundai Robotics robots in ROS2.<br>**hdr.urdf.xacro**: Top-level macro including all components.<br>**hdr_transmissions.xacro**: Defines joint transmissions.<br>**hdr.ros2_control.xacro**: Includes ros2_control macros.<br>**robots/**: Model-specific sub-directories with robot XACROs (e.g., `ha006b.urdf.xacro`).<br>**tools/**: End-effector descriptions (grippers). |
| `meshes/`           | Contains collision and visual meshes for all models.                                  |
| `examples/`         | Example URDF xacro and launch files for robot with gripper combinations.             |

---

## 3. Installation

#### Build the package
##### Create a ROS2 workspace

```bash
# Skip if you already have a workspace
mkdir -p ~/ros2_ws/src/
```

##### Clone the latest repositories

```bash
cd ~/ros2_ws
git clone https://github.com/hyundai-robotics/hdr_description src
```

##### Install ROS2 dependencies

```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

##### Build and source environment
```bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 4. Usage

#### Example

##### Visualize a Robot in RViz
The following launch file enables robot control in RViz using a mock hardware interface provided by `mock_components/GenericSystem`.<br>
The joint_state_publisher_gui is launched, allowing manual control of the robot through the GUI.

```bash
ros2 launch hdr_description display_robot.launch.py robot_model:=ha006b
```

| Argument Name         | Type   | Default          | Description                                                                 |
|-----------------------|--------|------------------|-----------------------------------------------------------------------------|
| `robot_model`         | string | `ha006b`         | HDR robot model to use. Choose from `[ha006b, hdf7_9, hdf8_8, hdr10l_19, hdr20_17, hdr50_22, hdr220_26, hh020, hdr35_20]` |
| `description_package` | string | `hdr_description` | Package containing robot URDF/XACRO files                                  |
| `description_file`    | string | `hdr.urdf.xacro`  | Defines the robot's URDF/XACRO configuration file                          |


##### Examples for Visualize a Robot with Gripper (End-Effector)

This package provides example configurations that demonstrate how to attach various grippers to HDR robots.<br>
The `examples/` directory contains pre-configured URDF xacro files and launch files that combine the HDF7_9 robot with different end-effectors for visualization in RViz.

**Supported Grippers:**

| Gripper | Directory | Description |
|---------|-----------|-------------|
| Robotiq 2F-85 | `urdf/tools/robotiq_gripper/2f_85/` | Robotiq 2-Finger 85mm stroke gripper |
| Robotiq 2F-140 | `urdf/tools/robotiq_gripper/2f_140/` | Robotiq 2-Finger 140mm stroke gripper |
| OnRobot RG2 | `urdf/tools/onrobot_gripper/rg2/` | OnRobot RG2 electric gripper |
| OnRobot RG6 | `urdf/tools/onrobot_gripper/rg6/` | OnRobot RG6 electric gripper |

```bash
# Robotiq 2F-85 (default)
ros2 launch hdr_description display_hdf7_9_with_gripper.launch.py

# Robotiq 2F-140
ros2 launch hdr_description display_hdf7_9_with_gripper.launch.py gripper_type:=robotiq_2f_140

# OnRobot RG2
ros2 launch hdr_description display_hdf7_9_with_gripper.launch.py gripper_type:=onrobot_rg2

# OnRobot RG6
ros2 launch hdr_description display_hdf7_9_with_gripper.launch.py gripper_type:=onrobot_rg6
```

| Argument Name         | Type   | Default          | Description                                                                 |
|-----------------------|--------|------------------|-----------------------------------------------------------------------------|
| `gripper_type`        | string | `robotiq_2f_85`  | Gripper type. Choose from `[robotiq_2f_85, robotiq_2f_140, onrobot_rg2, onrobot_rg6]` |
| `description_package` | string | `hdr_description`| Package containing robot URDF/XACRO files                                   |
| `use_mock_hardware`   | string | `true`           | Use mock hardware for visualization                                         |

---

## Maintainers and License

- **Maintainer**: HD Hyundai Robotics R&D Team
- **License**: This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE)
- **Contact**: <vewry12@hd.com>

---

## Third-Party Licenses

This package includes URDF and mesh files from the following third-party projects:

| Component | Source | License |
|-----------|--------|---------|
| Robotiq Gripper (2F-85, 2F-140) | [PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper) | BSD-3-Clause |
| OnRobot Gripper (RG2, RG6) | [ABC-iRobotics/onrobot-ros2](https://github.com/ABC-iRobotics/onrobot-ros2) | MIT |
