cmake_minimum_required(VERSION 3.8)
project(cpp_divider)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/DivisionInputFloats.msg"
)

# generate message
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# export message runtime dependency
ament_export_dependencies(rosidl_default_runtime)

set(cpp_source_files
  src/cpp_divider_node.cpp
  src/main.cpp
)

add_executable(${PROJECT_NAME}_node ${cpp_source_files})
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_14)  # Require C99 and C++14
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs)

# link generated message source code
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  set(test_source_files
    src/cpp_divider_node.cpp
    test/main.cpp
    test/cpp_divider_node_test.cpp
  )

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test ${test_source_files})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    std_msgs
  )

  # link generated message source code
  rosidl_target_interfaces(${PROJECT_NAME}_test
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

ament_package()
