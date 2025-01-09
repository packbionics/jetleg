// Copyright 2025 Pack Bionics
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


#ifndef PLANNING__MOVE_GROUP_PLANNER_HPP_
#define PLANNING__MOVE_GROUP_PLANNER_HPP_


#include <moveit/move_group_interface/move_group_interface.h>

#include <string>
#include <vector>
#include <memory>

#include <planning/planner_interface.hpp>

class MoveGroupPlanner : public PlannerInterface
{
public:
  virtual ~MoveGroupPlanner();

  void init(rclcpp::Node::SharedPtr node, std::string planning_group);
  virtual bool setGoal(const std::vector<double> & positions);

  moveit::core::MoveItErrorCode getStatusCode();

  void plan() override;
  void execute() override;

private:
  moveit::core::MoveItErrorCode mStatusCode;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> mPlan;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mMoveGroupIface;
};

#endif  // PLANNING__MOVE_GROUP_PLANNER_HPP_
