#include "node/finite_state_controller_node.hpp"

FinStateCtrlNode::FinStateCtrlNode(const FinStateCtrlPtr& controller)
{    
    mController = controller;

    mNode = std::make_shared<rclcpp::Node>("jetleg_planner");
}

void FinStateCtrlNode::doStateTransitionCallback(const TransReqPtr request, TransRespPtr response)
{
    NodePtr node = getNode();

    RCLCPP_INFO(node->get_logger(), "Transitionining to next state");
}

FinStateCtrlNode::NodePtr FinStateCtrlNode::getNode()
{
    return mNode;
}