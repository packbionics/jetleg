import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.client import Client

from rcl_interfaces.srv import GetParameters

from jetleg_control.gait_mode import GaitPhase
from jetleg_control.classifier_parameters import rule_based_classifier


def init_subplots(num_plots, subplot_description):
    fig, ax = plt.subplots(num_plots)

    if num_plots == 1:
        subplot_title = subplot_description[0] + ' vs. ' + subplot_description[1]

        ax.set_title(subplot_title)

        ax.set_xlabel(subplot_description[1])
        ax.set_ylabel(subplot_description[0])

        return fig, ax

    for i in range(num_plots):
        subplot_title = subplot_description[i][0] + ' vs. ' + subplot_description[i][1]

        ax[i].set_title(subplot_title)

        ax[i].set_xlabel(subplot_description[i][1])
        ax[i].set_ylabel(subplot_description[i][0])

    return fig, ax

def plot(data_plots, fig, ax):
    # display.clear_output(wait=True)
    # display.display(plt.gcf())
    # plt.clf()

    fig.suptitle('Training...')

    num_plots = len(data_plots)
    if num_plots == 1:
        data = data_plots[0]
        ax.plot(data)
    else:
        for i in range(num_plots):
            data = data_plots[i]
            ax[i].plot(data)
    
    plt.pause(0.1)

def add_client(node: Node, srv_type, srv_name) -> Client:
    """Helper function for creating ROS client and waiting for service availability"""

    client = node.create_client(srv_type, srv_name)
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    return client

from typing import List

def gen_gait_phases(params: rule_based_classifier.Params, joint_names: List[str]) -> List[GaitPhase]:
    
    phases = list()
    num_joints = len(joint_names)

    # Iterate over each gait phase
    for idx in range(0, len(params.stiffness), num_joints):

        # Access parameters for each phase at a time
        phase_stiffness = params.stiffness[idx: idx + num_joints]
        phase_damping = params.damping[idx: idx + num_joints]
        phase_equilibrum = params.equilibrium[idx: idx + num_joints]

        current_phase = GaitPhase(phase_stiffness, phase_damping, phase_equilibrum)
        phases.append(current_phase)

    return phases

def query_joint_names(node, client):
    # Retrieve list of controlled joints
    joint_param_request = GetParameters.Request()

    joint_param_request.names = ["joints"]

    future = client.call_async(joint_param_request)
    rclpy.spin_until_future_complete(node, future)

    parameter_values = future.result().values
    return parameter_values[0].string_array_value
