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


#include <moveit/move_group_interface/move_group_interface.h>

#include <controller/finite_state_controller.hpp>
#include <node/finite_state_controller_node.hpp>

#include "jetleg_planner_parameters.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("jetleg_planner");

int main(int argc, char ** argv)
{
  // Create a ROS 2 node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  std::shared_ptr<FinStateCtrlNode> finStateCtrlNode = std::make_shared<FinStateCtrlNode>();
  auto move_group_node = finStateCtrlNode->getNode();

  // Load any structured parameters
  auto param_listener = std::make_shared<jetleg_planner::ParamListener>(move_group_node);
  auto params = param_listener->get_params();

  // Next get the current set of joint values for the group.
  std::vector<std::vector<double>> phase_positions;
  RCLCPP_INFO(LOGGER, "Processing joint poses...");

  // Loop over each described joint pose
  for (const auto & pose_entry : params.config.joint_positions_map) {
    std::string joint_pose_name = pose_entry.first;
    auto joint_pose_config_mapping = pose_entry.second.joints_map;


    RCLCPP_INFO(LOGGER, "Extracting joint pose: %s ...", joint_pose_name.c_str());
    std::vector<double> joint_position;

    // Retrieve joint angles for each joint to describe a given pose
    for (const auto & joint_value_entry : joint_pose_config_mapping) {
      std::string joint_name = joint_value_entry.first;
      double joint_angle = joint_value_entry.second.value;

      joint_position.push_back(joint_angle);
    }

    phase_positions.push_back(joint_position);
  }

  // Set a controller to handle gait phase transitions
  std::shared_ptr<FinStateCtrl> finiteStateController = std::make_shared<FinStateCtrl>(
    phase_positions, 0);
  finStateCtrlNode->setController(finiteStateController);

  // MoveGroupInterface Setup
  static const std::string PLANNING_GROUP = "jetleg_leg";
  moveit::planning_interface::MoveGroupInterface & move_group = finStateCtrlNode->getMoveGrpIface();

  // Getting Basic Information
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(
    move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    std::ostream_iterator<std::string>(std::cout, ", "));

  // Spin the ROS 2 node
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
