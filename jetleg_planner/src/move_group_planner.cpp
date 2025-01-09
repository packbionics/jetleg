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


#include "planning/move_group_planner.hpp"


MoveGroupPlanner::~MoveGroupPlanner()
{}

void MoveGroupPlanner::init(rclcpp::Node::SharedPtr node, std::string planning_group)
{
  mMoveGroupIface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node,
    planning_group);
}

bool MoveGroupPlanner::setGoal(const std::vector<double> & positions)
{
  return mMoveGroupIface->setJointValueTarget(positions);
}

moveit::core::MoveItErrorCode MoveGroupPlanner::getStatusCode()
{
  return mStatusCode;
}

void MoveGroupPlanner::plan()
{
  mStatusCode = mMoveGroupIface->plan(*mPlan);
}

void MoveGroupPlanner::execute()
{
  mStatusCode = mMoveGroupIface->execute(*mPlan);
}
