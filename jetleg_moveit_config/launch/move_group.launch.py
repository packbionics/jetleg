from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jetleg", package_name="jetleg_moveit_config").to_moveit_configs()
    moveit_config.planning_pipelines['use_sim_time'] = True

    # try:
    #     print(moveit_config.to_dict()['use_sim_time'])
    # except ValueError as e:
    #     pass

    return generate_move_group_launch(moveit_config)
