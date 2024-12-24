cmake_minimum_required(VERSION 3.8)
project(ndi_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ndi_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# External libraries (NDI CAPI)
set(EXT_LIBS_DIR external)
file(GLOB NDICAPI_LIBRARY_SRC ${EXT_LIBS_DIR}/ndi_capi/src/*.cpp)

add_library(
    ndicapi_library
    SHARED
    ${NDICAPI_LIBRARY_SRC}
)

target_include_directories(ndicapi_library
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${EXT_LIBS_DIR}/ndi_capi/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${EXT_LIBS_DIR}/ndi_capi/src/include>
    $<INSTALL_INTERFACE:include>
)

# Main library (NDI Driver)
add_library(
    ${PROJECT_NAME}
    SHARED
    src/ndi_driver.cpp
)

# Update include directories for main library
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${EXT_LIBS_DIR}/ndi_capi/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${EXT_LIBS_DIR}/ndi_capi/src/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/${EXT_LIBS_DIR}
)

# First link with libraries that don't use ament
target_link_libraries(${PROJECT_NAME} ndicapi_library yaml-cpp)

# Then add ROS dependencies
ament_target_dependencies(${PROJECT_NAME}
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    ndi_msgs
    ament_index_cpp
)

# Set target properties
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "NDI_HARDWARE_BUILDING_LIBRARY")
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface ndi_hardware_plugin.xml)

# Install targets and headers
install(
  TARGETS ${PROJECT_NAME} ndicapi_library
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files
install(
  DIRECTORY
    include/
    ${EXT_LIBS_DIR}/ndi_capi/include/
    ${EXT_LIBS_DIR}/ndi_capi/src/include/
  DESTINATION
    include
)

# Export everything needed for downstream packages
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
  ndicapi_library
)

ament_export_targets(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ndi_msgs
  ament_index_cpp
)

ament_package()