// Copyright 2024 Pack Bionics
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


#ifndef SERVICE__FINITE_STATE_CONTROLLER_SERVICE_HPP_
#define SERVICE__FINITE_STATE_CONTROLLER_SERVICE_HPP_


#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include <controller/finite_state_controller.hpp>
#include <typedef.hpp>

#include <planning/planner_interface.hpp>
#include "planning/move_group_planner.hpp"


/**
 * @brief Manages ROS 2 services or other channels related to finite state control
 *
 * An instance of this class generates reference to ROS 2 node during construction.
 */
class FinStateCtrlService
{
public:
  typedef std_srvs::srv::Empty TransitionSrv;

  typedef std::shared_ptr<TransitionSrv::Request> TransReqPtr;
  typedef std::shared_ptr<TransitionSrv::Response> TransRespPtr;

  /**
   * @brief Construct a new Finite State Controller Node object
   *
   * This constructor initializes the underlying controller as null
   */
  FinStateCtrlService();

  /**
   * @brief Construct a new Finite State Controller Node object
   *
   * @param controller reference to the associated Finite State Controller
   */
  explicit FinStateCtrlService(const FinStateCtrlPtr & controller);

  /**
   * @brief Handles requests to transition to the next state in the FSM
   * associated with the underlying controller
   *
   * @param request describes the request to transition from the client
   * @param response describes the response returned to the client
   */
  void doStateTransitionCallback(const TransReqPtr request, TransRespPtr response);

  /**
   * @brief Set the object used to determine the next desired pose
   *
   * @param controller object used to determine the next desired pose
   */
  void setController(FinStateCtrlPtr controller);

  /**
   * @brief Get the controller used to retrieve the next pose
   *
   * @return FinStateCtrlPtr
   */
  FinStateCtrlPtr getController();

  /**
   * @brief Get the Node object
   *
   * @return NodePtr Reference to the associated ROS 2 Node handle
   */
  NodePtr getNode();

  /**
   * @brief Assign a reference to a MoveGroupInterface
   *
   * @param interfacePtr reference to a MoveGroupInterface
   */
  void setMoveGroupIfacePtr(std::shared_ptr<MoveGroupPlanner> interfacePtr);

  /**
   * @brief Get the reference to an MoveGroupInterface object
   *
   * @return std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
   */
  std::shared_ptr<MoveGroupPlanner> getMoveGroupIfacePtr();

private:
  /** Reference to the associated Finite State Controller */
  FinStateCtrlPtr mController;

  /** Reference to the associated ROS 2 Node handle */
  NodePtr mNode;

  /** Reference to the Service for handling requests to transition state */
  rclcpp::Service<TransitionSrv>::SharedPtr mService;

  /** Reference to move_group interface */
  std::shared_ptr<MoveGroupPlanner> mMoveGroupPtr;

  const std::string SERVICE_NAME = "transition_leg_state";
  const std::string PLANNING_GROUP = "jetleg_leg";
};

#endif  // SERVICE__FINITE_STATE_CONTROLLER_SERVICE_HPP_
