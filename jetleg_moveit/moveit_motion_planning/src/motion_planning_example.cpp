#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "motion_plan_example", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("motion_plan_example");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "leg");

  // // Set a target Pose
  // auto const target_pose = [] {
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.w = 1.0;
  //   msg.position.x = 0.28;
  //   msg.position.y = -0.2;
  //   msg.position.z = 0.5;
  //   return msg;
  // }();
  // move_group_interface.setPoseTarget(target_pose);

  // Set a target joint state
  auto const target_joint_state = [] {
    std::map<std::string, double> joint_state;

    joint_state.insert(std::pair<std::string, double>("vertical_rail_to_mount", 25.0));
    joint_state.insert(std::pair<std::string, double>("knee_joint", 70.0));
    joint_state.insert(std::pair<std::string, double>("ankle_joint", 25.0));

    return joint_state;
  }();
  move_group_interface.setJointValueTarget(target_joint_state);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}