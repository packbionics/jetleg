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

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("jetleg_planner");

int main(int argc, char ** argv)
{
  // Next get the current set of joint values for the group.
  std::vector<std::vector<double>> phase_positions;

  phase_positions.push_back({0.0, 0.0});
  phase_positions.push_back({0.0, (1.0 / 15) * M_PI});
  phase_positions.push_back({(1.0 / 2) * M_PI, (2.5 / 180) * M_PI});
  phase_positions.push_back({0.0, (2.5 / 180) * M_PI});

  std::shared_ptr<FinStateCtrl> finiteStateController = std::make_shared<FinStateCtrl>(
    phase_positions, 0);

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  std::shared_ptr<FinStateCtrlNode> finStateCtrlNode = std::make_shared<FinStateCtrlNode>();
  auto move_group_node = finStateCtrlNode->getNode();

  auto param_listener = std::make_shared<jetleg_planner::ParamListener>(move_group_node);
  auto params = param_listener->get_params();

  finStateCtrlNode->setController(finiteStateController);
  
  // Setup
  static const std::string PLANNING_GROUP = "jetleg_leg";
  moveit::planning_interface::MoveGroupInterface & move_group = finStateCtrlNode->getMoveGrpIface();

  // Getting Basic Information
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(
    move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    std::ostream_iterator<std::string>(std::cout, ", "));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
