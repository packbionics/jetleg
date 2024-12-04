#ifndef FINITE_STATE_CONTROLLER_NODE_HPP
#define FINITE_STATE_CONTROLLER_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include <controller/finite_state_controller.hpp>
#include <typedef.hpp>

class FinStateCtrlNode
{
    typedef std_srvs::srv::Empty TransitionSrv;

    typedef std::shared_ptr<TransitionSrv::Request> TransReqPtr;
    typedef std::shared_ptr<TransitionSrv::Response> TransRespPtr;

public:

    /**
     * @brief Construct a new Finite State Controller Node object
     * 
     * @param controller reference to the associated Finite State Controller
     */
    FinStateCtrlNode(const FinStateCtrlPtr& controller);

    /**
     * @brief Handles requests to transition to the next state in the FSM
     * associated with the underlying controller
     * 
     * @param request describes the request to transition from the client
     * @param response describes the response returned to the client
     */
    void doStateTransitionCallback(const TransReqPtr request, TransRespPtr response);

    /**
     * @brief Get the Node object
     * 
     * @return NodePtr Reference to the associated ROS 2 Node handle
     */
    NodePtr getNode();
    
private:

    /** Reference to the associated Finite State Controller */
    FinStateCtrlPtr mController;

    /** Reference to the associated ROS 2 Node handle */
    NodePtr mNode;

    /** Reference to the Service for handling requests to transition state */
    rclcpp::Service<TransitionSrv>::SharedPtr mService;

    const std::string SERVICE_NAME = "transition_leg_state";
};

#endif // FINITE_STATE_CONTROLLER_NODE_HPP