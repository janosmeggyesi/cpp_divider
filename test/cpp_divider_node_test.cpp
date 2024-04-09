/// @file cpp_divider_node_test.cpp
/// @brief The implementation file for the test cases for the CppDividerNode class.

#include "cpp_divider/cpp_divider_node.hpp"

#include <gtest/gtest.h>

TEST(cpp_divider, divide_validParametersReturnsCorrectOutput) {
  constexpr double testDividend = 4.0;
  constexpr double testDivisor = 2.0;
  constexpr double expectedValue = 2.0;
  CppDividerNode node{"test_node", "input_topic", "output_topic"};

  ASSERT_DOUBLE_EQ(expectedValue, node.divide(testDividend, testDivisor));
}

TEST(cpp_divider, divide_invalidParametersThrowsException) {
  constexpr double testDividend = 4.0;
  constexpr double testDivisor = 0.0;
  CppDividerNode node{"test_node", "input_topic", "output_topic"};

  ASSERT_THROW(node.divide(testDividend, testDivisor), std::invalid_argument);
}
