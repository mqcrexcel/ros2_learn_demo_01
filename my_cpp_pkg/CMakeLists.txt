cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(cpp_1st_node src/hello_world_2nd.cpp)
add_executable(cpp_2nd_node src/hello_world_2nd.cpp)
add_executable(robot_dog_publisher src/robot_dog_publisher.cpp)
add_executable(ground_station src/station_subscriber.cpp)

ament_target_dependencies(cpp_1st_node rclcpp)
ament_target_dependencies(cpp_2nd_node rclcpp)
ament_target_dependencies(robot_dog_publisher rclcpp)
ament_target_dependencies(robot_dog_publisher example_interfaces)
ament_target_dependencies(ground_station rclcpp)
ament_target_dependencies(ground_station example_interfaces)

install(TARGETS 
  cpp_1st_node
  cpp_2nd_node
  robot_dog_publisher
  ground_station
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
