/// @file cpp_divider_node.cpp
/// @brief The implementation file for the CppDividerNode class.

#include "cpp_divider/cpp_divider_node.hpp"

CppDividerNode::CppDividerNode(const std::string & nodeName, const std::string & inputTopicName,
                               const std::string & resultTopicName)
    : Node(nodeName) {
  subscription_ = this->create_subscription<cpp_divider::msg::DivisionInputFloats>(
      inputTopicName, 10,
      std::bind(&CppDividerNode::subscriber_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<std_msgs::msg::Float64>(resultTopicName, 10);
}

double CppDividerNode::divide(const double dividend, const double divisor) {
  if (divisor == 0) {
    throw std::invalid_argument("Divisor cannot be zero.");
  }
  return dividend / divisor;
}

void CppDividerNode::subscriber_callback(
    const cpp_divider::msg::DivisionInputFloats::SharedPtr msg) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Received parameters:\n dividend: " << msg->dividend.data
                                                         << ", divisor: " << msg->divisor.data);

  try {
    auto result = std_msgs::msg::Float64{};
    result.data = divide(msg->dividend.data, msg->divisor.data);
    RCLCPP_INFO_STREAM(this->get_logger(), "Division was successful, result: " << result.data);
    publisher_->publish(result);
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Division was unsuccessful and returned an exception: " << e.what());
  }
}
