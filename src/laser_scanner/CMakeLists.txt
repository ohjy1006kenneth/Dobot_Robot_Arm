cmake_minimum_required(VERSION 3.8)
project(laser_scanner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)              
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration filters)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# small_gicp (header-only traits + built shared lib)
set(SMALL_GICP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../small_gicp")
include_directories(${SMALL_GICP_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(NOT DEFINED KSJ3D_INCLUDE_DIR)
  set(KSJ3D_INCLUDE_DIR "./include/KSJApi.inc")
endif()
if(NOT DEFINED KSJ3D_LIB)
  set(KSJ3D_LIB "./include/KSJApi.bin/x64")
endif()

include_directories(${KSJ3D_INCLUDE_DIR})

add_executable(laser_driver src/laser_driver.cpp)
ament_target_dependencies(laser_driver 
  rclcpp 
  sensor_msgs 
  std_msgs
  std_srvs
  pcl_conversions
)

add_executable(scan_accumulator_cpp src/scan_accumulator.cpp)
ament_target_dependencies(scan_accumulator_cpp
    rclcpp 
    sensor_msgs 
    std_srvs
    geometry_msgs
    tf2_ros
    tf2_eigen
    pcl_conversions
)
target_link_libraries(scan_accumulator_cpp
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    ${SMALL_GICP_DIR}/../../build/small_gicp/libsmall_gicp.so
)

add_executable(scanning_coordinator_cpp src/scanning_coordinator.cpp)
ament_target_dependencies(scanning_coordinator_cpp
    rclcpp 
    std_msgs
    std_srvs
    rcl_interfaces
)

add_executable(map_merger_cpp src/map_merger.cpp)
ament_target_dependencies(map_merger_cpp
    rclcpp
    sensor_msgs
    std_srvs
    geometry_msgs
    tf2_ros
    tf2_eigen
    pcl_conversions
    rcl_interfaces
)
target_include_directories(map_merger_cpp PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(map_merger_cpp
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    ${SMALL_GICP_DIR}/../../build/small_gicp/libsmall_gicp.so
)


target_include_directories(laser_driver PRIVATE ${PCL_INCLUDE_DIRS} ${KSJ3D_INCLUDE_DIR})
target_include_directories(scan_accumulator_cpp PRIVATE ${PCL_INCLUDE_DIRS})

set(KSJ_LIB ${PROJECT_SOURCE_DIR}/include/KSJApi.bin/x64/libKSJApi3D.so)

target_link_libraries(laser_driver
    ${KSJ_LIB}
    KSJApi3D
)

# Add RPATH so the executable can find libKSJApi3D.so at runtime
set_target_properties(laser_driver PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/include/KSJApi.bin/x64"
)

install(TARGETS laser_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    scan_accumulator_cpp
    scanning_coordinator_cpp
    map_merger_cpp
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
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

# ── Python nodes ──────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/scan_accumulator
    scripts/scanning_coordinator
    scripts/map_merger
    DESTINATION lib/${PROJECT_NAME}
)
# ──────────────────────────────────────────────────────────

ament_package()
