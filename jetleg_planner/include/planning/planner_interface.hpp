#ifndef PLANNING__PLANNER_INTERFACE_HPP_
#define PLANNING__PLANNER_INTERFACE_HPP_


#include <moveit/move_group_interface/move_group_interface.h>

class PlannerInterface
{
public:
    PlannerInterface();

    virtual void plan() = 0;
    virtual void execute() = 0;
};

#endif // PLANNING__PLANNER_INTERFACE_HPP_