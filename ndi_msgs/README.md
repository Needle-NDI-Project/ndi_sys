# NDI Messages Package

## Table of Contents

1. [Overview](#overview)
2. [Message Definitions](#message-definitions)
    - [Directory Structure](#directory-structure)
    - [Message Types](#message-types)
3. [Build System](#build-system)
    - [CMake Configuration](#cmake-configuration)
4. [Dependencies](#dependencies)
    - [Required Packages](#required-packages)
5. [Installation](#installation)
6. [Message Details](#message-details)
    - [RigidArray Message](#rigidarray-message)
    - [TrackerArray Message](#trackerarray-message)
7. [Usage Examples](#usage-examples)
    - [Publishing Messages](#publishing-messages)
    - [Subscribing to Messages](#subscribing-to-messages)
8. [Best Practices](#best-practices)
    - [Message Usage](#message-usage)
    - [Performance Considerations](#performance-considerations)
    - [Error Handling](#error-handling)
9. [Development Guide](#development-guide)
    - [Adding New Messages](#adding-new-messages)
    - [Message Evolution](#message-evolution)
    - [Testing](#testing)

## Overview

The NDI Messages package defines custom message types for NDI tracking systems. These messages provide standardized data structures for communicating tracking information within the ROS2 ecosystem.

### Key Features

- Standardized message definitions
- Timestamp synchronization
- Array-based data structures
- Compatibility with geometry_msgs
- Support for multiple trackers

## Message Definitions

### Directory Structure

```plaintext
ndi_msgs/
├── msg/
│   ├── RigidArray.msg
│   └── TrackerArray.msg
├── CMakeLists.txt
└── package.xml
```

### Message Types

#### RigidArray.msg

```plaintext
# Time of sensor data acquisition, coordinate frame ID
std_msgs/Header header

# Array of tracker IDs
int32[] ids

# Array of tracker names
string[] frames

# If tracker is within measurement pyramid
bool[] inbound

# Array of 3D poses in the frame given in header
geometry_msgs/Pose[] poses
```

#### TrackerArray.msg

```plaintext
# Time of sensor data acquisition, coordinate frame ID
std_msgs/Header header

# Array of point IDs
int32[] ids

# Array of 3d points in the frame given in header
geometry_msgs/Point32[] points

# Array of estimation residuals
float32[] residuals
```

## Build System

### CMake Configuration

```cmake
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackerArray.msg"
  "msg/RigidArray.msg"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)
```

## Dependencies

### Required Packages

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>builtin_interfaces</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>

<exec_depend>rosidl_default_runtime</exec_depend>
```

## Installation

1. Clone the package:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/Needle-NDI-Project/ndi_sys.git
    ```

2. Build messages:

    ```bash
    colcon build --packages-select ndi_msgs
    ```

3. Source the workspace:

    ```bash
    source install/setup.bash
    ```

## Message Details

### RigidArray Message

1. `header`: Standard ROS2 header
   - `stamp`: Time of data acquisition
   - `frame_id`: Reference frame

2. `ids`: Unique tracker identifiers
   - Type: `int32[]`
   - Purpose: Tracker identification

3. `frames`: Named reference frames
   - Type: `string[]`
   - Purpose: Frame identification

4. `inbound`: Tracking status flags
   - Type: `bool[]`
   - Purpose: Validity indication

5. `poses`: 6-DOF poses
   - Type: `geometry_msgs/Pose[]`
   - Components:
     - Position (x, y, z)
     - Orientation (quaternion)

### TrackerArray Message

1. `header`: Standard ROS2 header

2. `ids`: Point identifiers
   - Type: `int32[]`
   - Purpose: Point identification

3. `points`: 3D positions
   - Type: `geometry_msgs/Point32[]`
   - Components: (x, y, z)

4. `residuals`: Estimation quality
   - Type: `float32[]`
   - Purpose: Error metrics

## Usage Examples

### Publishing Messages

```cpp
#include "ndi_msgs/msg/rigid_array.hpp"

auto publisher = node->create_publisher<ndi_msgs::msg::RigidArray>(
    "rigid_poses", 10);

ndi_msgs::msg::RigidArray msg;
msg.header.stamp = node->now();
msg.header.frame_id = "ndi_frame";

// Add poses
geometry_msgs::msg::Pose pose;
pose.position.x = 1.0;
// ... set other fields ...
msg.poses.push_back(pose);

publisher->publish(msg);
```

### Subscribing to Messages

```cpp
void callback(const ndi_msgs::msg::RigidArray::SharedPtr msg)
{
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        const auto& pose = msg->poses[i];
        const auto& frame = msg->frames[i];
        const bool valid = msg->inbound[i];
        // Process data...
    }
}

auto subscription = node->create_subscription<ndi_msgs::msg::RigidArray>(
    "rigid_poses", 10, callback);
```

## Best Practices

### Message Usage

1. Always set header timestamp
2. Maintain array size consistency
3. Check inbound flags
4. Use appropriate frame IDs

### Performance Considerations

1. Pre-allocate vectors
2. Minimize message size
3. Use appropriate QoS settings
4. Monitor publishing frequency

### Error Handling

```cpp
if (msg->poses.size() != msg->ids.size()) {
    RCLCPP_ERROR(logger, "Inconsistent array sizes");
    return;
}

if (msg->poses.empty()) {
    RCLCPP_WARN(logger, "Empty pose array");
    return;
}
```

## Development Guide

### Adding New Messages

1. Create message definition file
2. Update CMakeLists.txt
3. Update package.xml
4. Build and test

### Message Evolution

- Maintain backward compatibility
- Use optional fields for extensions
- Document changes thoroughly

### Testing

```bash
# Verify message generation
ros2 interface show ndi_msgs/msg/RigidArray

# Monitor message statistics
ros2 topic info /rigid_poses

# Echo messages
ros2 topic echo /rigid_poses
```
