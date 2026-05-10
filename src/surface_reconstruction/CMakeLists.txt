cmake_minimum_required(VERSION 3.8)
project(surface_reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

# Nav2 arrival -> /start_repair trigger
add_executable(nav2_arrival_trigger src/nav2_arrival_trigger.cpp)
ament_target_dependencies(
  nav2_arrival_trigger
  rclcpp
  rclcpp_action
  action_msgs
  nav2_msgs
  std_msgs
  geometry_msgs
  unique_identifier_msgs
)
install(TARGETS nav2_arrival_trigger DESTINATION lib/${PROJECT_NAME})

# Reverse Ranger mount adapter: Nav2 uses the peripheral side as +X while
# the Ranger hardware driver keeps its native +X.
add_executable(reverse_ranger_mount_adapter src/reverse_ranger_mount_adapter.cpp)
ament_target_dependencies(
  reverse_ranger_mount_adapter
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS reverse_ranger_mount_adapter DESTINATION lib/${PROJECT_NAME})

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
