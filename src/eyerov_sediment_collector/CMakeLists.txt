cmake_minimum_required(VERSION 3.8)
project(eyerov_sediment_collector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eyerov_sediment_interfaces REQUIRED)

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

include_directories(include)

add_executable(sediment_collector_node
  src/main.cpp
  src/sediment_collector_node.cpp
)

ament_target_dependencies(sediment_collector_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  lifecycle_msgs
  std_msgs
  eyerov_sediment_interfaces
)

add_executable(mock_rov_hardware_node
  src/mock_rov_hardware_node.cpp
)

ament_target_dependencies(mock_rov_hardware_node
  rclcpp
  std_msgs
  eyerov_sediment_interfaces
)

install(TARGETS
  sediment_collector_node
  mock_rov_hardware_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()
