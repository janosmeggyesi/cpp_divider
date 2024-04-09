/// @file cpp_divider_node.h
/// @brief The header file for the CppDividerNode class.

#pragma once

#include <string>

#include "cpp_divider/msg/division_input_floats.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

/// @brief A rclcpp::Node implementation that can divide two numbers arriving from a topic and
/// provide a result an other one.
class CppDividerNode : public rclcpp::Node {
public:
  /// @brief Constructor.
  /// @param nodeName Name of the node.
  /// @param inputTopicName The name of the topic that the node can receive the input variables for
  /// division.
  /// @param resultTopicName The name of the topic the the node can provide the result of the
  /// division
  CppDividerNode(const std::string & nodeName, const std::string & inputTopicName,
                 const std::string & resultTopicName);

  /// @brief Implements division with error handling
  /// @param dividend The 64 bit length floating point number we want to divide.
  /// @param divisor The 64 bit length floating point number we want to divide.
  /// @return The 64 bit length floating point result of the division.
  double divide(const double dividend, const double divisor);

private:
  /// @brief Callback function for the division input subscription.
  /// @param msg Arrived message.
  void subscriber_callback(const cpp_divider::msg::DivisionInputFloats::SharedPtr msg);

  rclcpp::Subscription<cpp_divider::msg::DivisionInputFloats>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};
