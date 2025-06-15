cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(cpp_1st_node src/hello_world_2nd.cpp)
add_executable(cpp_2nd_node src/hello_world_2nd.cpp)
ament_target_dependencies(cpp_1st_node rclcpp)
ament_target_dependencies(cpp_2nd_node rclcpp)

add_executable(robot_dog_publisher src/robot_dog_publisher.cpp)
ament_target_dependencies(robot_dog_publisher rclcpp)
ament_target_dependencies(robot_dog_publisher std_msgs)

add_executable(ground_station src/station_subscriber.cpp)
ament_target_dependencies(ground_station rclcpp)
ament_target_dependencies(ground_station std_msgs)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp)
ament_target_dependencies(number_publisher std_msgs)
ament_target_dependencies(number_publisher std_srvs)

add_executable(number_station src/station_number.cpp)
ament_target_dependencies(number_station rclcpp)
ament_target_dependencies(number_station std_msgs)
ament_target_dependencies(number_station std_srvs)

add_executable(two_int_server src/two_int_server.cpp)
ament_target_dependencies(two_int_server rclcpp)
ament_target_dependencies(two_int_server example_interfaces)

add_executable(two_int_client src/two_int_client.cpp)
ament_target_dependencies(two_int_client rclcpp)
ament_target_dependencies(two_int_client example_interfaces)

install(TARGETS 
  cpp_1st_node
  cpp_2nd_node
  robot_dog_publisher
  ground_station
  number_publisher
  number_station
  two_int_server
  two_int_client
  DESTINATION lib/${PROJECT_NAME}
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
