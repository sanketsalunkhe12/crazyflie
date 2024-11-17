cmake_minimum_required(VERSION 3.8)
project(crazyflie_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Add -Wno-error=pedantic to prevent pedantic warnings from being treated as errors
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-error=pedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(crazyflie_msgs REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)

# crazyflie_cpp and tools
add_subdirectory(src/crazyflie_tools)
include_directories(src/crazyflie_tools/crazyflie_cpp/include
                    src/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/include
                    ${EIGEN3_INCLUDE_DIRS})

include_directories(include)

list(APPEND ROS2_LIBRARIES
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  std_srvs
  tf2_ros
  sensor_msgs
  crazyflie_msgs
  motion_capture_tracking_interfaces
)

add_executable(crazyflie_server src/crazyflie_server.cpp)
target_link_libraries(crazyflie_server crazyflie_cpp)
ament_target_dependencies(crazyflie_server ${ROS2_LIBRARIES})

add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop crazyflie_cpp)
ament_target_dependencies(teleop ${ROS2_LIBRARIES})

# Install C++ executables
install(TARGETS
        # crazyflie_tools
        comCheck
        scan
        listParams
        listLogVariables
        listMemories
        reboot
        battery
        version
        console
        log
        setParam
        downloadUSDLogfile
        #
        teleop
        crazyflie_server
        DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/crazyflie_server.py
  scripts/chooser.py
  scripts/vel_mux.py
  scripts/cfmult.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        config
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
