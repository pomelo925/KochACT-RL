import os
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction, SetEnvironmentVariable, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Load config yaml file
    config_path = os.path.join(
        get_package_share_directory('koch_ros2_wrapper'), 'config', 'd415.yaml'
    )
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    # Set colorized output
    colorize_action = SetEnvironmentVariable(
        name='RCUTILS_COLORIZED_OUTPUT', 
        value='1'
    )
    
    # Get shared settings
    shared_settings = config["shared_settings"]
    color_profile = shared_settings["color_profile"]
    depth_profile = shared_settings["depth_profile"]
    align_depth = shared_settings.get("align_depth", False)
    enable_depth = shared_settings.get("enable_depth", False)
    enable_infra1 = shared_settings.get("enable_infra1", False)
    enable_infra2 = shared_settings.get("enable_infra2", False)
    initial_reset = shared_settings.get("initial_reset", False) 
    log_level = shared_settings.get("log_level", "warn")

    # Launch actions
    launch_actions = [colorize_action]

    print_customized_logs = False

    # Print shared settings once
    if print_customized_logs:
        shared_settings_msg = f"\n[ Shared settings ]\n{shared_settings}\n"
        launch_actions.append(LogInfo(msg=shared_settings_msg))

    for idx, cam in enumerate(config["cameras"]):
        if not cam.get("enabled", True):
            continue

        params = {
            "serial_no": cam["serial_no"],
            "camera_name": cam["camera_name"],
            "rgb_camera.color_profile": color_profile,
            "depth_module.depth_profile": depth_profile,
            "align_depth": align_depth,
            "enable_depth": enable_depth,
            "enable_infra1": enable_infra1,
            "enable_infra2": enable_infra2,
            "initial_reset": initial_reset,
        }

        # Print customized settings
        if print_customized_logs:
            customized_settings_msg = f"\n[ CAM '{cam['camera_name']}' PARAMS]\n{params}\n==="
            launch_actions.append(LogInfo(msg=customized_settings_msg))  

        delay = 1.0  # delay between launching cameras (s)
        node_action = TimerAction(
            period=idx * delay,  
            actions=[Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                namespace=cam["name"],
                parameters=[params],
                arguments=['--ros-args', '--log-level', log_level],
                output='log'
            )]
        )

        launch_actions.append(node_action)

    return LaunchDescription(launch_actions)
