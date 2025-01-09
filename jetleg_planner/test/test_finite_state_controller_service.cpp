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


// Note: Includes boiler-plate code to mimic what Google Testing framework
// should do automatically
class FakeMoveGroupIFace : public MoveGroupPlanner
{
public:
  FakeMoveGroupIFace()
  {
    set_goal_called_n_times = 0;
    plan_called_n_times = 0;
    execute_called_n_times = 0;
  }

  bool setGoal(const std::vector<double> &) override {set_goal_called_n_times++; return true;}
  void plan() override {plan_called_n_times++;}
  void execute() override {execute_called_n_times++;}

  unsigned int getNumberTimesSetGoalCalled() {return set_goal_called_n_times;}
  unsigned int getNumberTimesPlanCalled() {return plan_called_n_times;}
  unsigned int getNumberTimesExecuteCalled() {return execute_called_n_times;}

private:
  unsigned int set_goal_called_n_times;
  unsigned int plan_called_n_times;
  unsigned int execute_called_n_times;
};

// TODO(agbrown6): Make this work
// class MockMoveGroupIFace : public FakeMoveGroupIFace
// {
// public:
//   MOCK_METHOD(bool, setGoal, (const std::vector<double> &), (override));
//   MOCK_METHOD(void, plan, (), (override));
//   MOCK_METHOD(void, execute, (), (override));
// };

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

  auto move_group_ptr = std::make_shared<FakeMoveGroupIFace>();

  // Give the service access to plan the motion of the move group
  finStateCtrlService->setMoveGroupIfacePtr(move_group_ptr);

  FinStateCtrlService::TransReqPtr req_msg;
  FinStateCtrlService::TransRespPtr resp_msg;

  finStateCtrlService->doStateTransitionCallback(req_msg, resp_msg);

  // Should check that move_group_ptr->setGoal(testing::_) runs exactly once
  // EXPECT_CALL(*move_group_ptr, setGoal(testing::_))
  // .Times(1);
  EXPECT_EQ(1, move_group_ptr->getNumberTimesSetGoalCalled());
  EXPECT_EQ(1, move_group_ptr->getNumberTimesPlanCalled());
  EXPECT_EQ(1, move_group_ptr->getNumberTimesExecuteCalled());

  // EXPECT_CALL(*move_group_ptr, plan())
  // .Times(1);
  // EXPECT_CALL(*move_group_ptr, execute())
  // .Times(1);

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
