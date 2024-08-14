#ifndef JETLEG_CONTROLLER__JETLEG_CONTROLLER_HPP_
#define JETLEG_CONTROLLER__JETLEG_CONTROLLER_HPP_

#include <rclcpp_action/rclcpp_action.hpp>

#include <controller_interface/controller_interface.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>


namespace jetleg_controller {

using FollowJointTrajectoryAction = control_msgs::action::FollowJointTrajectory;

class JetlegController : public controller_interface::ControllerInterface
{
public:
  JetlegController();

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
    rclcpp_action::GoalResponse goal_received_callback(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectoryAction::Goal>);

    rclcpp_action::CancelResponse goal_cancelled_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>> goal_handle);

    void goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>> goal_handle);

    rclcpp_lifecycle::LifecycleNode::SharedPtr mNode;
    rclcpp_action::Server<FollowJointTrajectoryAction>::SharedPtr mActionServer;
};

}

#endif  // JETLEG_CONTROLLER__JETLEG_CONTROLLER_HPP_