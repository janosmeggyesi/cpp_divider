/// @file main.cpp
/// @brief The entrypoint of the program.

#include "cpp_divider/cpp_divider_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<CppDividerNode>("cpp_divider_node", "input_numbers", "division_result"));
  rclcpp::shutdown();
  return 0;
}
