#include "planning/move_group_planner.hpp"


void MoveGroupPlanner::init(rclcpp::Node::SharedPtr node, std::string planning_group)
{
    mMoveGroupIface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, planning_group);
}

void MoveGroupPlanner::plan()
{
    mMoveGroupIface->plan(*mPlan);
}

void MoveGroupPlanner::execute()
{
    mMoveGroupIface->execute(*mPlan);
}