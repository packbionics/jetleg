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


// #include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <service/finite_state_controller_service.hpp>


class MockMoveGroupIFace : public moveit::planning_interface::MoveGroupInterface
{
public:
  MockMoveGroupIFace(rclcpp::Node::SharedPtr a, std::string b)
  : moveit::planning_interface::MoveGroupInterface(a, b)
  {}

  MOCK_METHOD(bool, setJointValueTarget, (std::vector<double>));
  MOCK_METHOD(
    moveit::core::MoveItErrorCode, plan,
    (moveit::planning_interface::MoveGroupInterface::Plan));
  MOCK_METHOD(
    moveit::core::MoveItErrorCode, execute,
    (moveit::planning_interface::MoveGroupInterface::Plan));
};

static int ARGC;
static char ** ARGV;

static void setup()
{
  int argc = ARGC;
  char ** argv = ARGV;

  // Create a ROS 2 node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
}

static void tearDown()
{
  rclcpp::shutdown();
}

TEST(finite_state_controller_service, test_constructor)
{
  setup();

  // Scenario: Construct a ROS 2 node with a valid reference to a MoveGroupInterface
  std::shared_ptr<FinStateCtrlService> finStateCtrlNode =
    std::make_shared<FinStateCtrlService>();
  auto move_group_node = finStateCtrlNode->getNode();

  EXPECT_NE(move_group_node, nullptr);

  tearDown();
}

TEST(finite_state_controller_service, test_set_controller)
{
  setup();

  // Scenario: Construct a ROS 2 node with a valid reference to a MoveGroupInterface
  static const std::vector<double> expected_pose_1 = {0.0, 0.0};
  static const std::vector<double> expected_pose_2 = {0.5, 1.0};

  static const std::vector<std::vector<double>> joint_pose_seq_1 =
  {expected_pose_1, expected_pose_2};
  FinStateCtrlPtr ptr = std::make_shared<FinStateCtrl>(joint_pose_seq_1, 0);

  std::shared_ptr<FinStateCtrlService> finStateCtrlService =
    std::make_shared<FinStateCtrlService>();
  auto node_ptr = finStateCtrlService->getNode();
  finStateCtrlService->setController(ptr);

  // MoveGroupInterface Setup
  static const std::string PLANNING_GROUP = "jetleg_leg";

  auto move_group_ptr = std::make_shared<MockMoveGroupIFace>(
    node_ptr,
    PLANNING_GROUP);

  // Give the service access to plan the motion of the move group
  finStateCtrlService->setMoveGroupIfacePtr(move_group_ptr);

  tearDown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  testing::InitGoogleMock(&argc, argv);

  ARGC = argc;
  ARGV = argv;

  return RUN_ALL_TESTS();
}
