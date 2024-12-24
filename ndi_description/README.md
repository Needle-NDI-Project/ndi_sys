# NDI Description Package

## Table of Contents

1. [Overview](#overview)
2. [Package Structure](#package-structure)
3. [URDF/XACRO Configuration](#urdfxacro-configuration)
4. [ROS2 Control Configuration](#ros2-control-configuration)
    - [Hardware Interface Configuration](#hardware-interface-configuration)
    - [Sensor Macro Definition](#sensor-macro-definition)
5. [SROM Management](#srom-management)
    - [SROM File Structure](#srom-file-structure)
    - [SROM File Loading](#srom-file-loading)
6. [Dependencies](#dependencies)
7. [Installation](#installation)
8. [Usage](#usage)
    - [Loading System Description](#loading-system-description)
    - [Configuring New Trackers](#configuring-new-trackers)
9. [Configuration Details](#configuration-details)
    - [Hardware Parameters](#hardware-parameters)
    - [Tracker Configuration](#tracker-configuration)
    - [Frame Definitions](#frame-definitions)
10. [Troubleshooting](#troubleshooting)
    - [Common Issues](#common-issues)
    - [Debugging Tools](#debugging-tools)
    - [Configuration Validation](#configuration-validation)

## Overview

The NDI Description package provides robot description files and configurations for NDI tracking systems. It includes URDF/XACRO descriptions, ROS2 Control configurations, and SROM file management for NDI trackers.

## Package Structure

```plaintext
ndi_description/
├── config/
│   └── ndi_system.urdf.xacro    # Main system description
├── ros2_control/
│   ├── ndi_system.ros2_control.xacro  # Control configuration
│   └── sensor_macro.xacro       # Sensor definitions
├── srom/                        # Tracker ROM files
│   ├── 8700338.rom
│   ├── 8700339.rom
│   ├── 8700340.rom
│   ├── UfoGuideTransducer.rom
│   └── active-wireless.rom
├── CMakeLists.txt              # Build configuration
└── package.xml                 # Package manifest
```

## URDF/XACRO Configuration

### System Description (ndi_system.urdf.xacro)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="ndi_system">
  <!-- Import NDI ros2_control description -->
  <xacro:include filename="$(find ndi_description)/ros2_control/ndi_system.ros2_control.xacro" />

  <link name="world" />
  <xacro:ndi_sensor name="aurora" />
</robot>
```

## ROS2 Control Configuration

### Hardware Interface Configuration

```xml
<ros2_control name="${name}" type="sensor">
  <hardware>
    <plugin>ndi_driver/NdiDriver</plugin>
    <param name="ndi_ip">192.155.1.80</param>
  </hardware>

  <!-- Tracker configurations -->
  <xacro:tracker name="fus_tracker"
                srom="UfoGuideTransducer.rom" />
  <xacro:tracker name="needle"
                srom="8700340.rom" />
</ros2_control>
```

### Sensor Macro Definition

```xml
<xacro:macro name="tracker" params="name srom">
  <sensor name="${name}">
    <state_interface name="pose.position.x" />
    <state_interface name="pose.position.y" />
    <state_interface name="pose.position.z" />
    <state_interface name="pose.orientation.x" />
    <state_interface name="pose.orientation.y" />
    <state_interface name="pose.orientation.z" />
    <state_interface name="pose.orientation.w" />
    <param name="srom">${srom}</param>
  </sensor>
</xacro:macro>
```

## SROM Management

### SROM File Structure

```plaintext
srom/
├── 8700338.rom             # Tracker ROM file
├── 8700339.rom             # Tracker ROM file
├── 8700340.rom             # Tracker ROM file
├── UfoGuideTransducer.rom  # Transducer ROM
└── active-wireless.rom     # Wireless tool ROM
```

### SROM File Loading

- Files are loaded during system initialization
- Automatic tool detection and configuration
- Error handling for missing/corrupt files

## Dependencies

### Required Packages

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>urdf</exec_depend>
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
    colcon build --packages-select ndi_description
    ```

4. Source workspace:

    ```bash
    source install/setup.bash
    ```

## Usage

### Loading System Description

```bash
# Using launch file
ros2 launch ndi_bringup ndi_system.launch.py

# Manual URDF loading
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro /path/to/ndi_system.urdf.xacro)"
```

### Configuring New Trackers

1. Add SROM file to `srom/` directory
2. Update ros2_control configuration:

```xml
<xacro:tracker name="new_tracker"
              srom="new_tracker.rom" />
```

## Configuration Details

### Hardware Parameters

```xml
<!-- NDI System Configuration -->
<hardware>
  <plugin>ndi_driver/NdiDriver</plugin>
  <param name="ndi_ip">192.155.1.80</param>
  <!-- Additional parameters as needed -->
</hardware>
```

### Tracker Configuration

```xml
<!-- Tracker Definition -->
<sensor name="${name}">
  <!-- Position interfaces -->
  <state_interface name="pose.position.x" />
  <state_interface name="pose.position.y" />
  <state_interface name="pose.position.z" />

  <!-- Orientation interfaces -->
  <state_interface name="pose.orientation.x" />
  <state_interface name="pose.orientation.y" />
  <state_interface name="pose.orientation.z" />
  <state_interface name="pose.orientation.w" />

  <!-- SROM configuration -->
  <param name="srom">${srom}</param>
</sensor>
```

### Frame Definitions

```xml
<!-- World frame definition -->
<link name="world" />

<!-- Sensor frame configuration -->
<xacro:ndi_sensor name="aurora">
  <!-- Sensor-specific configuration -->
</xacro:ndi_sensor>
```

## Troubleshooting

### Common Issues

1. SROM File Problems

    ```plaintext
    Problem: SROM file not found
    Solution:
    - Verify file exists in srom/ directory
    - Check file permissions
    - Validate file integrity
    ```

2. Configuration Errors

    ```plaintext
    Problem: Invalid URDF/XACRO
    Solution:
    - Check XML syntax
    - Verify file paths
    - Validate parameters
    ```

3. Hardware Interface Issues

    ```plaintext
    Problem: Missing interfaces
    Solution:
    - Check interface definitions
    - Verify sensor configuration
    - Review hardware parameters
    ```

### Debugging Tools

```bash
# Validate URDF
check_urdf /path/to/generated.urdf

# View URDF contents
xacro /path/to/ndi_system.urdf.xacro

# List available interfaces
ros2 control list_hardware_interfaces
```

### Configuration Validation

```bash
# Check URDF syntax
xacro --xacro-file /path/to/ndi_system.urdf.xacro --check-cycles

# Verify ros2_control configuration
ros2 control list_hardware_components
```
