#include "jetleg_controller/jetleg_controller.hpp"

namespace jetleg_controller {

  enum class ERROR_TYPE
  {
    PARAM_LOAD_FAIL,
  };

  static const std::map<ERROR_TYPE, const std::string> ERROR_MSG_TEMPLATES = {
    {ERROR_TYPE::PARAM_LOAD_FAIL,
      "Exception thrown during init stage with message: %s \n"},
  };

  static const std::string NAMESPACE_SEPARATOR = "/";
  static const std::string TOPIC_NAME = "follow_joint_trajectory";


  controller_interface::CallbackReturn JetlegController::on_init()
  {
    try
    {
      // Create the parameter listener and get the parameters
      mParamListener = std::make_shared<jetleg_controller::ParamListener>(get_node());
      mParams = mParamListener->get_params();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), ERROR_MSG_TEMPLATES.at(ERROR_TYPE::PARAM_LOAD_FAIL).c_str(), e.what());
      return CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration JetlegController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for(const auto & joint : mParams.joints) {
      for (const auto & cmd_if : mParams.command_interfaces) {
        conf.names.push_back(joint + NAMESPACE_SEPARATOR + cmd_if);
      }
    }

    return conf;
  }

  controller_interface::InterfaceConfiguration JetlegController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Specify joint state interfaces from parameters
    for(const auto & joint : mParams.joints) {
      for (const auto & cmd_if : mParams.state_interfaces) {
        conf.names.push_back(joint + NAMESPACE_SEPARATOR + cmd_if);
      }
    }
    
    // Specify IMU sensor state interfaces from parameters
    std::map<std::string, semantic_components::IMUSensor> imuSensors;
    for(const auto & imu : mParams.imus) {
      const auto [entry, success] = imuSensors.insert({imu, semantic_components::IMUSensor(imu)});
      std::vector<std::string> stateInterfaceNames = entry->second.get_state_interface_names();

      conf.names.insert(conf.names.end(), stateInterfaceNames.begin(), stateInterfaceNames.end());
    }

    return conf;
  }

  controller_interface::CallbackReturn JetlegController::on_configure(
    const rclcpp_lifecycle::State & /* previous_state */)
  {

    // Create the action server which will handle motion control requests
    mActionServer = rclcpp_action::create_server<FollowJointTrajectoryAction>(
      get_node(),
      std::string(get_node()->get_name()) + TOPIC_NAME,
      std::bind(&JetlegController::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JetlegController::goal_cancelled_callback, this, std::placeholders::_1),
      std::bind(&JetlegController::goal_accepted_callback, this, std::placeholders::_1)
    );

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn JetlegController::on_activate(
    const rclcpp_lifecycle::State & /* previous_state */)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn JetlegController::on_deactivate(
    const rclcpp_lifecycle::State & /* previous_state */)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }


  controller_interface::return_type JetlegController::update(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    return controller_interface::return_type::OK;
  }

  rclcpp_action::GoalResponse JetlegController::goal_received_callback(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectoryAction::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse JetlegController::goal_cancelled_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>> /* goal_handle */)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void JetlegController::goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>> /* goal_handle */)
  {

  }


}  // jetleg_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  jetleg_controller::JetlegController, controller_interface::ControllerInterface)