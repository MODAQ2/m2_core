
import yaml
import os
import shutil
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessStart
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

# Load config
configName = 'm1_m2_comparison_config.yaml'
config = os.path.join(
    get_package_share_directory('m2_launch'),
    'config',
    configName
)
cfg = yaml.safe_load(open(config))
logPath = cfg["M2Supervisor"]["ros__parameters"]["loggerPath"]

# Save config for record
systemStartTime = datetime.utcnow().strftime("%Y_%m_%d_%H_%M_%S")
shutil.copy(config, os.path.join(logPath, f"{systemStartTime}_{configName}"))

print("ROS_DOMAIN_ID: ", os.environ['ROS_DOMAIN_ID'])
def generate_launch_description():
    m2_supervisor_node = Node(
        package="m2_supervisor",
        executable="m2_supervisor_node",
        name="M2Supervisor",
        parameters=[config],
        output="both",
        respawn=True
    )

    dependent_nodes = [
        Node(
            package="bag_recorder",
            executable="bag_recorder_node",
            parameters=[config],
            output="both",
            respawn=True
        ),
        Node(
            package="labjack_t8_ros2",
            executable="labjack_ain_streamer",
            name="Labjack1",
            parameters=[config],
            output="both",
            respawn=True
        ),
        Node(
            package="labjack_t8_ros2",
            executable="labjack_ain_streamer",
            name="Labjack2",
            parameters=[config],
            output="both",
            respawn=True
        )
    ]

    # Timer delay (e.g., 5 seconds) after M2Supervisor starts
    delayed_start = TimerAction(
        period=2.0,  # seconds
        actions=dependent_nodes
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='9'),

        m2_supervisor_node,

        RegisterEventHandler(
            OnProcessStart(
                target_action=m2_supervisor_node,
                on_start=[delayed_start]
            )
        )
    ])
