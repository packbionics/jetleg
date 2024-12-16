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


#include "node/finite_state_controller_node.hpp"

FinStateCtrlNode::FinStateCtrlNode()
{
  mNode = std::make_shared<rclcpp::Node>("jetleg_planner");

  mMoveGroupPtr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    mNode,
    PLANNING_GROUP);

  // Get a reference to the service callback
  //
  // Since the callback is a member function, the function needs to be
  // provided with 'this' as its first argument.
  //
  // Placeholders are used in-place of the arbitrary service arguments
  auto serviceRef = std::bind(
    &FinStateCtrlNode::doStateTransitionCallback, this,
    std::placeholders::_1, std::placeholders::_2);
  mService = mNode->create_service<TransitionSrv>(SERVICE_NAME, serviceRef);

  mController = nullptr;
}

FinStateCtrlNode::FinStateCtrlNode(const FinStateCtrlPtr & controller)
{
  FinStateCtrlNode();
  setController(controller);
}

void FinStateCtrlNode::doStateTransitionCallback(const TransReqPtr request, TransRespPtr response)
{
  NodePtr node = getNode();
  rclcpp::Logger LOGGER = node->get_logger();

  // Check if a controller has been assigned to the node
  if (mController == nullptr) {
    RCLCPP_ERROR(LOGGER, "Controller has not been set. This client request shall be ignored.");
    return;
  }

  auto & move_group = getMoveGrpIface();

  RCLCPP_INFO(LOGGER, "Transitionining to next state...");

  // Stores the subsequent joint positions into the given vector
  std::vector<double> joint_group_positions;
  mController->next(joint_group_positions);

  // Assigns the vector of joint positions as the goal/target
  bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
  if (!within_bounds) {
    RCLCPP_WARN(
      LOGGER,
      "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

  move_group.execute(my_plan);
}

void FinStateCtrlNode::setController(FinStateCtrlPtr controller)
{
  mController = controller;
}

NodePtr FinStateCtrlNode::getNode()
{
  return mNode;
}

moveit::planning_interface::MoveGroupInterface & FinStateCtrlNode::getMoveGrpIface()
{
  return *mMoveGroupPtr;
}
