#include "jetleg_controller/jetleg_controller.hpp"

namespace jetleg_controller {

  JetlegController::JetlegController()
  {
    mNode = get_node();
  }

  controller_interface::CallbackReturn JetlegController::on_init()
  {
    if(controller_interface::ControllerInterface::on_init() != controller_interface::CallbackReturn::SUCCESS) {
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration()
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    conf.names.push_back("knee_joint_/position");

    return conf;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration()
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    conf.names.push_back("knee_joint_/position");

    conf.names.push_back("imu0/orientation.x");
    conf.names.push_back("imu0/orientation.y");
    conf.names.push_back("imu0/orientation.z");
    conf.names.push_back("imu0/orientation.w");

    conf.names.push_back("imu0/angular_velocity.x");
    conf.names.push_back("imu0/angular_velocity.y");
    conf.names.push_back("imu0/angular_velocity.z");

    conf.names.push_back("imu0/linear_acceleration.x");
    conf.names.push_back("imu0/linear_acceleration.y");
    conf.names.push_back("imu0/linear_acceleration.z");

    return conf;
  }

  controller_interface::CallbackReturn JetlegController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
  {
    mActionServer = rclcpp_action::create_server<FollowJointTrajectoryAction>(
      mNode,
      std::string(mNode->get_name()) + "follow_joint_trajectory",
      std::bind(&JetlegController::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JetlegController::goal_cancelled_callback, this, std::placeholders::_1),
      std::bind(&JetlegController::goal_accepted_callback, this, std::placeholders::_1)
    );
  }

  controller_interface::CallbackReturn JetlegController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn JetlegController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }


  controller_interface::return_type JetlegController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    return controller_interface::return_type::OK;
  }

  rclcpp_action::GoalResponse goal_received_callback(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectoryAction::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse goal_cancelled_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>> goal_handle)
  {

  }


}  // jetleg_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  jetleg_controller::JetlegController, controller_interface::ControllerInterface)