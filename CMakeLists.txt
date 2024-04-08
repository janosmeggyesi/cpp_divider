cmake_minimum_required(VERSION 3.8)
project(cpp_divider)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/DivisionInputFloats.msg"
)

# generate message
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# export message runtime dependency
ament_export_dependencies(rosidl_default_runtime)

set(cpp_source_files
  src/cpp_divider_node.cpp
)

add_executable(cpp_divider_node ${cpp_source_files})
ament_target_dependencies(cpp_divider_node rclcpp std_msgs)

# link generated message source code
rosidl_target_interfaces(cpp_divider_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS cpp_divider_node
  DESTINATION lib/${PROJECT_NAME})

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
