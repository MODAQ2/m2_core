# Launch file for m2
import yaml
import os
import shutil
from launch import LaunchDescription
from launch_ros.actions import  Node
from launch.actions import SetEnvironmentVariable
from launch_ros.actions.node import ExecuteProcess
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

#### PATH TO MODAQ CONFIG FILE ######
## FYI THIS PULLS THE FILE FROM THE INSTALL DIRECTORY, NOT THE SRC DIRECTORY
configName = 'm1_m2_comparison_config.yaml'
config = os.path.join(
      get_package_share_directory('m2_launch'),
      'config',
      configName
      )

cfg = yaml.safe_load(open(config))
logPath = cfg["M2Supervisor"]["ros__parameters"]["loggerPath"]

systemStartTime = datetime.utcnow().strftime("%Y_%m_%d_%H_%M_%S")
## Copy config file for later review
shutil.copy(config, logPath+"/"+systemStartTime+"_"+configName)

print("ROS_DOMAIN_ID: ", os.environ['ROS_DOMAIN_ID'])
def generate_launch_description():

  return LaunchDescription([
    SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='9'),

    Node(
    package    = "bag_recorder",
    executable = "bag_recorder_node",
    ## Automatically gets named BagRecorded - do not add name= ""
    parameters = [config],
    output="both",
    respawn = True
    ),
    Node(
    package    = "labjack_t8_ros2",
    executable = "labjack_ain_streamer",
    name       = "Labjack1",
    parameters = [config],
    output="both",
    respawn = True
    ),
    Node(
    package    = "labjack_t8_ros2",
    executable = "labjack_ain_streamer",
    name       = "Labjack2",
    parameters = [config],
    output="both",
    respawn = True
    ),
    Node(
    package    = "m2_supervisor",
    executable = "m2_supervisor_node",
    name      = "M2Supervisor",
    parameters = [config],
    output="both",
    respawn = True
    )

  ]
                           
   )

