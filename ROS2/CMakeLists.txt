cmake_minimum_required(VERSION 3.5)
project(other_components)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(Target_attitude src/Target_attitude_node.cpp)
ament_target_dependencies(Target_attitude rclcpp std_msgs)

add_executable(Test_command src/Test_command_node.cpp)
ament_target_dependencies(Test_command rclcpp std_msgs)

add_executable(Reaction_wheel src/Reaction_wheel_node.cpp)
ament_target_dependencies(Reaction_wheel rclcpp std_msgs)

add_executable(Rigid_body src/Rigid_body_node.cpp)
ament_target_dependencies(Rigid_body rclcpp std_msgs)

add_executable(Gyro_sensor src/Gyro_sensor_node.cpp)
ament_target_dependencies(Gyro_sensor rclcpp std_msgs)

add_executable(Star_Tracker_sensor src/Star_Tracker_sensor_node.cpp)
ament_target_dependencies(Star_Tracker_sensor rclcpp std_msgs)

add_executable(Kalman_filter src/Kalman_filter_node.cpp)
ament_target_dependencies(Kalman_filter rclcpp std_msgs)

add_executable(PD_controller src/PD_controller_node.cpp)
ament_target_dependencies(PD_controller rclcpp std_msgs)

add_executable(Socket_client src/Socket_client.cpp)
ament_target_dependencies(Socket_client rclcpp std_msgs)

add_executable(Simple_socket_client src/Simple_socket_client.cpp)
ament_target_dependencies(Simple_socket_client rclcpp std_msgs)

add_executable(Simple_socket_server src/Simple_socket_server.cpp)
ament_target_dependencies(Simple_socket_server rclcpp std_msgs)



install(TARGETS

  Target_attitude
  Test_command

  Reaction_wheel
  Rigid_body
  Gyro_sensor
  Star_Tracker_sensor
  Kalman_filter
  PD_controller

  Socket_client
  Simple_socket_client
  Simple_socket_server

  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
