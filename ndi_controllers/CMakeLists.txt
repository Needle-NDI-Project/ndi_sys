cmake_minimum_required(VERSION 3.8)
project(ndi_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ndi_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/ndi_pose_broadcaster.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    include
)

ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  controller_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  ndi_msgs
  geometry_msgs
  Eigen3
  eigen3_cmake_module
)

# Prevent BOOST from being used
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  controller_interface
  rclcpp_lifecycle
  ndi_msgs
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()