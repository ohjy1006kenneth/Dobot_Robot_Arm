cmake_minimum_required(VERSION 3.8)
project(laser_scanner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)              
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration filters)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

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

add_executable(scan_accumulator src/scan_accumulator.cpp)
ament_target_dependencies(scan_accumulator 
    rclcpp 
    sensor_msgs 
    std_srvs
    geometry_msgs
    tf2_ros
    tf2_sensor_msgs
    pcl_conversions
)
target_link_libraries(scan_accumulator ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(scanning_coordinator src/scanning_coordinator.cpp)
ament_target_dependencies(scanning_coordinator 
    rclcpp 
    std_msgs
    std_srvs
)


target_include_directories(laser_driver PRIVATE ${PCL_INCLUDE_DIRS} ${KSJ3D_INCLUDE_DIR})
target_include_directories(scan_accumulator PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(scan_accumulator ${PCL_LIBRARIES})

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
    laser_driver
    scan_accumulator
    scanning_coordinator
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

ament_package()
