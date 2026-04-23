cmake_minimum_required(VERSION 3.8)
project(surface_reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# IMU unit relay node (g -> m/s²)
add_executable(imu_g_to_ms2_relay src/imu_g_to_ms2_relay.cpp)
ament_target_dependencies(imu_g_to_ms2_relay rclcpp sensor_msgs)
install(TARGETS imu_g_to_ms2_relay DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch urdf config map
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
