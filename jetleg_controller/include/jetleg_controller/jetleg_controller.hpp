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


#ifndef JETLEG_CONTROLLER__JETLEG_CONTROLLER_HPP_
#define JETLEG_CONTROLLER__JETLEG_CONTROLLER_HPP_

#include <memory>
#include <rclcpp_action/rclcpp_action.hpp>

#include <controller_interface/controller_interface.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <semantic_components/imu_sensor.hpp>

#include "jetleg_controller_parameters.hpp"


namespace jetleg_controller
{

using FollowJointTrajectoryAction = control_msgs::action::FollowJointTrajectory;

class JetlegController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;


  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectoryAction::Goal>);

  rclcpp_action::CancelResponse goal_cancelled_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>> goal_handle);

  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>> goal_handle);

  rclcpp_action::Server<FollowJointTrajectoryAction>::SharedPtr mActionServer;

  std::shared_ptr<jetleg_controller::ParamListener> mParamListener;
  jetleg_controller::Params mParams;
};

}  // namespace jetleg_controller

#endif  // JETLEG_CONTROLLER__JETLEG_CONTROLLER_HPP_
