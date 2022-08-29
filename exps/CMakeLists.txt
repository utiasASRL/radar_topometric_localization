cmake_minimum_required(VERSION 3.16)
project(exps)

## Common setup for vtr packages
include("$ENV{VTRSRC}/main/src/vtr_common/vtr_include.cmake")

## Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_lidar REQUIRED)
find_package(vtr_radar REQUIRED)
find_package(vtr_radar_lidar REQUIRED)

## C++ Libraries

## Executables
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# lidar
add_executable(${PROJECT_NAME}_lidar_preprocessing src/lidar_preprocessing.cpp)
ament_target_dependencies(${PROJECT_NAME}_lidar_preprocessing rclcpp std_msgs vtr_common vtr_logging vtr_lidar)
add_executable(${PROJECT_NAME}_lidar_odometry src/lidar_odometry.cpp)
ament_target_dependencies(${PROJECT_NAME}_lidar_odometry rclcpp std_msgs vtr_common vtr_logging vtr_lidar)
add_executable(${PROJECT_NAME}_lidar_localization src/lidar_localization.cpp)
ament_target_dependencies(${PROJECT_NAME}_lidar_localization rclcpp std_msgs vtr_common vtr_logging vtr_lidar)

# radar
add_executable(${PROJECT_NAME}_radar_preprocessing src/radar_preprocessing.cpp)
ament_target_dependencies(${PROJECT_NAME}_radar_preprocessing rclcpp std_msgs vtr_common vtr_logging vtr_radar)
add_executable(${PROJECT_NAME}_radar_odometry src/radar_odometry.cpp)
ament_target_dependencies(${PROJECT_NAME}_radar_odometry rclcpp std_msgs vtr_common vtr_logging vtr_radar)
add_executable(${PROJECT_NAME}_radar_localization src/radar_localization.cpp)
ament_target_dependencies(${PROJECT_NAME}_radar_localization rclcpp std_msgs vtr_common vtr_logging vtr_radar)

# radar lidar
add_executable(${PROJECT_NAME}_radar_lidar_localization src/radar_lidar_localization.cpp)
ament_target_dependencies(${PROJECT_NAME}_radar_lidar_localization rclcpp std_msgs vtr_common vtr_logging vtr_radar_lidar)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_lidar_preprocessing
    ${PROJECT_NAME}_lidar_odometry
    ${PROJECT_NAME}_lidar_localization
    ${PROJECT_NAME}_radar_preprocessing
    ${PROJECT_NAME}_radar_odometry
    ${PROJECT_NAME}_radar_localization
    ${PROJECT_NAME}_radar_lidar_localization
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
