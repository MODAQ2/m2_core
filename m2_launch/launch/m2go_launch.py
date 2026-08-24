# Launch file for m2go
import yaml
import os
from launch import LaunchDescription
from launch_ros.actions import  Node
from launch_ros.actions.node import ExecuteProcess
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
import shutil


#### PATH TO MODAQ CONFIG FILE ######
## FYI THIS PULLS THE FILE FROM THE INSTALL DIRECTORY, NOT THE SRC DIRECTORY
configName = 'm2go_config.yaml'
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

dataPath = f"{cfg['BagRecorder']['ros__parameters']['dataFolder']}/Bag_{systemStartTime}"

def generate_launch_description():

  return LaunchDescription([
    Node(
    package    = "bag_recorder",
    executable = "bag_recorder_node",
    ## Automatically gets named BagRecorded - do not add name= ""
    parameters = [config],
    output="screen",
    respawn = True
    ),
    Node(
    package    = "m2_supervisor",
    executable = "m2_supervisor_node",
    name = "M2Supervisor",
    parameters = [config],
    output="screen",
    respawn = True
    ),
    Node(
    package    = "labjack_t8_ros2",
    executable = "labjack_ain_streamer",
    name = "LabjackAINFast",
    parameters = [config],
    output="log",
    respawn = True
    ), 
    Node(
    package    = "labjack_t8_ros2",
    executable = "labjack_ain_reader",
    name = "LabjackAINSlow",
    parameters = [config],
    output="log",
    respawn = True
    ),    
    Node(
    package    = "labjack_t8_ros2",
    executable = "labjack_do_node",
    name = "LabjackDO",
    parameters = [config],
    output="screen",
    respawn = True
    ),
    Node(
    package    = "labjack_t8_ros2",
    executable = "labjack_dac_writer",
    name = "LabjackDAC",
    parameters = [config],
    output="screen",
    respawn = True
    ),   
    Node(
    package    = "labjack_t8_ros2",
    executable = "labjack_dio_reader",
    name = "LabjackDIN",
    parameters = [config],
    output="screen",
    respawn = True
    ),
    # Node(
    # package='rosbridge_server_m2',
    # executable='rosbridge_websocket',
    # name='Rosbridge',
    # parameters=[config],
    # ),
    # Node(
    # package='rosapi',
    # executable='rosapi_node',
    # ),
    Node(
    name = 'AdnavCompass',
    package = 'adnav_driver',
    executable = 'adnav_driver',
    emulate_tty = True,
    output = 'log',
    arguments=['--ros-args', '--log-level', 'ERROR'],
    parameters = [config]
    ),
    Node(
    package='bluespace_ai_xsens_mti_driver',
    executable='xsens_mti_node',
    name='XsensINS',
    output='screen',
    parameters=[config],
    arguments=[]
    ),
    Node(
    package='ed582',
    executable='ed582_driver',
    name='RTDed582',
    output='screen',
    parameters=[config],
    arguments=[]
    ),
    # Node(
    # package='m2_control',
    # executable='m2_control',
    # name='M2Control',
    # output='screen',
    # parameters=[config],
    # arguments=[]
    # ),
    # Node(
    # package='parquet_writer',
    # executable='parquet_write_consumer',
    # name='M2parquet',
    # output='screen',
    # parameters=[config],
    # arguments=[]
    # )
  ]
                           
   )

