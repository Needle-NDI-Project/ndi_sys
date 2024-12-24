# NDI System

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
   - [Package Structure](#package-structure)
   - [Component Overview](#component-overview)
3. [Prerequisites](#prerequisites)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Configuration](#configuration)
    - [Device Setup](#device-setup)
    - [Tracker Configuration](#tracker-configuration)
    - [Controller Setup](#controller-setup)
7. [Technical Details](#technical-details)
8. [Troubleshooting](#troubleshooting)
9. [License](#license)
10. [Authors](#authors)
11. [Acknowledgments](#acknowledgments)

## Overview

This ROS2 package provides a comprehensive integration for Northern Digital Inc. (NDI) optical tracking systems, with specific support for the Polaris and Aurora series. The system enables high-precision 6-DOF tracking of rigid bodies using optical markers and integrates seamlessly with the ROS2 control framework.

## System Architecture

### Package Structure

```plaintext
ndi_sys/
├── ndi_driver/                 # Hardware interface implementation
│   ├── external/               # NDI CAPI integration
│   ├── include/                # Public headers
│   └── src/                    # Implementation files
├── ndi_controllers/            # ROS2 controllers
├── ndi_msgs/                   # Message definitions
├── ndi_description/            # URDF and configurations
└── ndi_bringup/                # Launch files and scripts
```

### Component Overview

#### 1. NDI Driver (`ndi_driver`)

- Core hardware interface implementation
- NDI CAPI integration
- Device communication handling
- Data acquisition and transformation

See more details in the [ndi_driver README](ndi_driver/README.md).

#### 2. NDI Controllers (`ndi_controllers`)

- Position and orientation broadcasting
- Real-time data publishing
- State interface management
- ROS2 Control integration

See more details in the [ndi_controllers README](ndi_controllers/README.md).

#### 3. NDI Messages (`ndi_msgs`)

Custom message types:

```plaintext
RigidArray.msg
└── Header header
└── int32[] ids
└── string[] frames
└── bool[] inbound
└── geometry_msgs/Pose[] poses

TrackerArray.msg
└── Header header
└── int32[] ids
└── geometry_msgs/Point32[] points
└── float32[] residuals
```

See more details in the [ndi_msgs README](ndi_msgs/README.md).

#### 4. NDI Description (`ndi_description`)

- URDF/XACRO system descriptions
- Tracker configurations
- ROS2 Control parameter definitions
- SROM file management

See more details in the [ndi_description README](ndi_description/README.md).

#### 5. NDI Bringup (`ndi_bringup`)

- Launch files
- System configuration
- Controller setup
- Runtime parameters

See more details in the [ndi_bringup README](ndi_bringup/README.md).

## Prerequisites

### Hardware

- NDI Polaris or Aurora tracking system
- Ethernet connection to device
- Supported trackers with ROM files

### Software

- Ubuntu 22.04 or later
- ROS2 Humble or later
- C++14 compatible compiler
- CMake 3.8+

### ROS2 Dependencies

```xml
<depend>hardware_interface</depend>
<depend>controller_interface</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>geometry_msgs</depend>
```

## Installation

1. Create workspace and clone repository:

    ```bash
    mkdir -p ndi_ws/src
    cd ndi_ws/src
    git clone https://github.com/Needle-NDI-Project/ndi_sys.git
    ```

2. Install dependencies:

    ```bash
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the workspace:

    ```bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

4. Source the workspace:

    ```bash
    source install/setup.bash
    ```

## Configuration

### Device Setup

1. Configure device IP in `ndi_description/config/ndi_system.urdf.xacro`:

```xml
<param name="ndi_ip">192.155.1.80</param>
```

### Tracker Configuration

1. Place SROM files in `ndi_description/srom/`
2. Configure trackers in `ndi_description/ros2_control/ndi_system.ros2_control.xacro`:

```xml
<xacro:tracker name="fus_tracker"
               srom="UfoGuideTransducer.rom" />
```

### Controller Setup

Edit `ndi_bringup/config/controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 10

ndi_broadcaster:
  ros__parameters:
    state_publish_rate: 10
    sensor_names:
      - fus_tracker
      - needle
    sensor_ids:
      - 1998
      - 2023
```

## Usage

### Launch System

```bash
ros2 launch ndi_bringup ndi_system.launch.py
```

### Monitor Data

```bash
# View tracking data
ros2 topic echo /rigid_poses

# List active controllers
ros2 control list_controllers
```

### Runtime Control

```bash
# Start tracking
ros2 control switch_controllers --start ndi_broadcaster

# Stop tracking
ros2 control switch_controllers --stop ndi_broadcaster
```

## Technical Details

### Coordinate Systems

- Internal: millimeter-scale measurements
- ROS2: meter-scale measurements
- Quaternion orientation: [q₀, qₓ, qᵧ, qᵣ]

### Performance

- Default update rate: 10Hz (configurable)
- Typical latency: ~100ms
- Position accuracy: sub-millimeter (device-dependent)
- Orientation accuracy: <0.5° (device-dependent)

## Troubleshooting

### Common Issues

1. Connection Errors

    ```plaintext
    Error: Failed to connect to device at 192.155.1.80
    Solution: Check IP address and network connectivity
    ```

2. SROM Loading Errors

    ```plaintext
    Error: Could not find SROM file: tracker.rom
    Solution: Verify SROM file path and permissions
    ```

3. Tracking Issues

    ```plaintext
    Error: No tools enabled
    Solution: Check tracker visibility and ROM configuration
    ```

### Debugging Tools

```bash
# Controller status
ros2 control list_controllers

# Hardware interface status
ros2 topic echo /controller_manager/status
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Authors

- [Shashank Goyal](mailto:sgoyal18@jhu.edu)

## Acknowledgments

- ICube Laboratory, University of Strasbourg
- Northern Digital Inc. for CAPI support
- Maciej Bednarczyk **[Original Author]**
- Adnan SAOOD **[Original Author]**
