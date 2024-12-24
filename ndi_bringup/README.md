# NDI Bringup Package

## Table of Contents

1. [Overview](#overview)
2. [Package Structure](#package-structure)
3. [Launch Files](#launch-files)
    - [ndi_system.launch.py](#ndi_systemlaunchpy)
    - [Launch Sequence](#launch-sequence)
4. [Configuration](#configuration)
    - [controllers.yaml](#controllersyaml)
5. [Dependencies](#dependencies)
    - [Required Packages](#required-packages)
6. [Installation](#installation)
7. [Usage](#usage)
    - [Basic Launch](#basic-launch)
    - [With Custom Configuration](#with-custom-configuration)
8. [Controller Configuration](#controller-configuration)
    - [Controller Types](#controller-types)
    - [Controller Parameters](#controller-parameters)
9. [Launch System](#launch-system)
    - [Launch File Components](#launch-file-components)
    - [Launch Sequence Management](#launch-sequence-management)
10. [Troubleshooting](#troubleshooting)
    - [Common Issues](#common-issues)
    - [Debugging Tools](#debugging-tools)
    - [Logging Configuration](#logging-configuration)

## Overview

The NDI Bringup package provides launch files and configuration for initializing NDI tracking systems in a ROS2 environment. It orchestrates the startup sequence of various components including the hardware interface, controllers, and state publishers.

## Package Structure

```plaintext
ndi_bringup/
├── config/
│   └── controllers.yaml     # Controller configuration
├── launch/
│   └── ndi_system.launch.py # Main launch file
├── CMakeLists.txt          # Build configuration
└── package.xml             # Package manifest
```

## Launch Files

### ndi_system.launch.py

The main launch file coordinates system startup with the following components:

1. **Robot State Publisher**
   - Loads URDF/XACRO descriptions
   - Publishes TF transformations

2. **Controller Manager**
   - Manages hardware interface
   - Loads controller configurations
   - Handles lifecycle transitions

3. **Controllers**
   - Joint State Broadcaster
   - NDI Pose Broadcaster
   - Configurable update rates

### Launch Sequence

```python
# Component initialization order
1. Robot State Publisher
2. Controller Manager
3. Joint State Broadcaster
4. NDI Pose Broadcaster
```

## Configuration

### controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 10  # Hz

joint_state_broadcaster:
  ros__parameters:
    state_publish_rate: 10
    state_interface_names:
      - pose.position.x
      - pose.position.y
      - pose.position.z
      - pose.orientation.x
      - pose.orientation.y
      - pose.orientation.z
      - pose.orientation.w

ndi_broadcaster:
  ros__parameters:
    state_publish_rate: 10
    sensor_names:
      - fus_tracker
      - needle
    sensor_ids:
      - 1998
      - 2023
    world_frame: ndi_frame
```

## Dependencies

### Required Packages

```xml
<exec_depend>ndi_driver</exec_depend>
<exec_depend>ndi_description</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>controller_manager</exec_depend>
<exec_depend>ros2_controllers</exec_depend>
<exec_depend>xacro</exec_depend>
```

## Installation

1. Add to workspace:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/your-repo/ndi_sys.git
    ```

2. Install dependencies:

    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build:

    ```bash
    colcon build --packages-select ndi_bringup
    ```

4. Source workspace:

    ```bash
    source install/setup.bash
    ```

## Usage

### Basic Launch

```bash
ros2 launch ndi_bringup ndi_system.launch.py
```

### With Custom Configuration

```bash
ros2 launch ndi_bringup ndi_system.launch.py
```

## Controller Configuration

### Controller Types

1. Joint State Broadcaster
   - Publishes joint states
   - Handles pose interfaces

2. NDI Pose Broadcaster
   - Publishes tracker poses
   - Manages sensor data

### Controller Parameters

```yaml
# Update rates
state_publish_rate: 10  # Hz

# Interface configuration
state_interface_names:
  - pose.position.x
  - pose.position.y
  # ... other interfaces ...

# Sensor configuration
sensor_names:
  - fus_tracker
  - needle
```

## Launch System

### Launch File Components

```python
# Robot description loading
robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([
        FindPackageShare("ndi_description"),
        "config",
        "ndi_system.urdf.xacro",
    ])
])

# Controller manager configuration
controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        robot_description,
        controller_config_file,
    ]
)

# Controller spawners
joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"]
)
```

### Launch Sequence Management

```python
# Delayed start using event handlers
delay_joint_state_broadcaster = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=controller_manager,
        on_start=[joint_state_broadcaster_spawner],
    )
)
```

## Troubleshooting

### Common Issues

1. Controller Loading Failures

    ```plaintext
    Problem: Controllers fail to load
    Solution:
    - Check controller configuration
    - Verify hardware interface availability
    - Ensure correct parameter values
    ```

2. Launch Errors

    ```plaintext
    Problem: Launch file fails
    Solution:
    - Verify package dependencies
    - Check file paths
    - Validate URDF/XACRO
    ```

3. Controller Start Issues

    ```plaintext
    Problem: Controllers won't start
    Solution:
    - Check controller manager status
    - Verify hardware connection
    - Review error messages
    ```

### Debugging Tools

```bash
# List controllers
ros2 control list_controllers

# Check controller status
ros2 control list_hardware_interfaces

# Monitor controller manager
ros2 topic echo /controller_manager/status
```

### Logging Configuration

```bash
# Enable debug logging
ros2 launch ndi_bringup ndi_system.launch.py --ros-args --log-level debug
```
