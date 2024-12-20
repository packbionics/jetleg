// Copyright 2024 Pack Bionics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include <gtest/gtest.h>

#include <controller/finite_state_controller.hpp>

TEST(finite_state_controller, test_constructor)
{
  // Scenario: Empty collection of joint poses
  EXPECT_THROW(FinStateCtrl({}, 0), std::invalid_argument);

  // Scenario: Singleton of joint poses
  EXPECT_NO_THROW(FinStateCtrl({{0.0, 0.0}}, 0));

  // Scenario: Multiple joint poses
  EXPECT_NO_THROW(FinStateCtrl({{0.0, 0.0}, {0.0, 0.0}}, 0));

  // Scenario: Inconsistent number of joints among joint poses
  EXPECT_THROW(FinStateCtrl({{0.0, 0.0, 0.0}, {0.0, 0.0}}, 0), std::invalid_argument);

  // Scenario: Negative starting index
  EXPECT_NO_THROW(FinStateCtrl({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, -1));

  // Scenario: Starting index higher than the number of poses
  EXPECT_NO_THROW(FinStateCtrl({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, 5));

  // Scenario: Starting index equal to the number of poses
  EXPECT_NO_THROW(FinStateCtrl({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, 2));

  // Scenario: Valid starting index greater than 0 to the number of poses
  EXPECT_NO_THROW(FinStateCtrl({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, 1));
}

TEST(finite_state_controller, test_next)
{
  static const std::vector<double> expected_pose_1 = {0.0, 0.0};
  static const std::vector<double> expected_pose_2 = {0.5, 1.0};

  static const std::vector<std::vector<double>> joint_pose_seq_1 = {expected_pose_1};
  static const std::vector<std::vector<double>> joint_pose_seq_2 =
  {expected_pose_1, expected_pose_2};

  FinStateCtrlPtr ptr;

  std::vector<double> current_pose;

  // Scenario: Singleton of joint poses
  EXPECT_NO_THROW(ptr = std::make_shared<FinStateCtrl>(joint_pose_seq_1, 0));

  // Should remain at the same pose after transitioning once
  ptr->next(current_pose);
  for (size_t i = 0; i < std::min(current_pose.size(), expected_pose_1.size()); i++) {
    EXPECT_DOUBLE_EQ(current_pose[i], expected_pose_1[i]);
  }

  // Should continue to remain at the same pose after any number of transitions
  ptr->next(current_pose);
  for (size_t i = 0; i < std::min(current_pose.size(), expected_pose_1.size()); i++) {
    EXPECT_DOUBLE_EQ(current_pose[i], expected_pose_1[i]);
  }

  // Scenario: Multiple joint poses
  EXPECT_NO_THROW(ptr = std::make_shared<FinStateCtrl>(joint_pose_seq_2, 0));

  // One transition to send it to the second pose
  ptr->next(current_pose);
  for (size_t i = 0; i < std::min(current_pose.size(), expected_pose_2.size()); i++) {
    EXPECT_DOUBLE_EQ(current_pose[i], expected_pose_2[i]);
  }

  // The poses should cycle back to the start
  ptr->next(current_pose);
  for (size_t i = 0; i < std::min(current_pose.size(), expected_pose_1.size()); i++) {
    EXPECT_DOUBLE_EQ(current_pose[i], expected_pose_1[i]);
  }

  // Scenario: Negative starting index
  EXPECT_NO_THROW(ptr = std::make_shared<FinStateCtrl>(joint_pose_seq_2, -1));

  // Starting at the last pose should transition to the first pose in the cycle
  ptr->next(current_pose);
  for (size_t i = 0; i < std::min(current_pose.size(), expected_pose_1.size()); i++) {
    EXPECT_DOUBLE_EQ(current_pose[i], expected_pose_1[i]);
  }

  // Scenario: Starting index higher than the number of poses
  EXPECT_NO_THROW(ptr = std::make_shared<FinStateCtrl>(joint_pose_seq_2, 5));

  // The index should be reduced according to the modulo operation
  ptr->next(current_pose);
  for (size_t i = 0; i < std::min(current_pose.size(), expected_pose_1.size()); i++) {
    EXPECT_DOUBLE_EQ(current_pose[i], expected_pose_1[i]);
  }

  // Scenario: Starting index equal to the number of poses
  EXPECT_NO_THROW(ptr = std::make_shared<FinStateCtrl>(joint_pose_seq_2, 2));

  // Similar situation as: index > # poses
  ptr->next(current_pose);
  for (size_t i = 0; i < std::min(current_pose.size(), expected_pose_2.size()); i++) {
    EXPECT_DOUBLE_EQ(current_pose[i], expected_pose_2[i]);
  }

  // Scenario: Valid starting index greater than 0 to the number of poses
  EXPECT_NO_THROW(ptr = std::make_shared<FinStateCtrl>(joint_pose_seq_2, 1));

  ptr->next(current_pose);
  for (size_t i = 0; i < std::min(current_pose.size(), expected_pose_1.size()); i++) {
    EXPECT_DOUBLE_EQ(current_pose[i], expected_pose_1[i]);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
