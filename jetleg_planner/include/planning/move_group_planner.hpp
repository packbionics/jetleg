#ifndef PLANNING__MOVE_GROUP_PLANNER_HPP_
#define PLANNING__MOVE_GROUP_PLANNER_HPP_


#include <moveit/move_group_interface/move_group_interface.h>

#include <planning/planner_interface.hpp>

class MoveGroupPlanner : public PlannerInterface
{
public:
    MoveGroupPlanner();

    void init(rclcpp::Node::SharedPtr node, std::string planning_group);

    void plan() override;
    void execute() override;
private:

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> mPlan;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mMoveGroupIface;
};

#endif // PLANNING__MOVE_GROUP_PLANNER_HPP_