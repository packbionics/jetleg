#include "node/finite_state_controller_node.hpp"

FinStateCtrlNode::FinStateCtrlNode(const FinStateCtrlPtr& controller)
{    
    mController = controller;

    mNode = std::make_shared<rclcpp::Node>("jetleg_planner");
}

void FinStateCtrlNode::doStateTransitionCallback(const TransReqPtr request, TransRespPtr response)
{
    NodePtr node = getNode();

    RCLCPP_INFO(node->get_logger(), "Transitionining to next state...");

    // std::vector<double> joint_group_positions;
    // mController->next(joint_group_positions);

    // // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
    // bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
    // if (!within_bounds)
    // {
    //   RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    // }

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // move_group.execute(my_plan);
}

NodePtr FinStateCtrlNode::getNode()
{
    return mNode;
}