#! /usr/bin/python3

import launch
import launch_ros
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node
import subprocess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("naoqi_driver")
    ip_declare = launch.actions.DeclareLaunchArgument(
        'nao_ip', default_value="127.0.0.1", description='Ip address of the robot'
    )
    nao_ip = launch.substitutions.LaunchConfiguration('nao_ip')
    wake_up_script = os.path.join(pkg_dir, "launch", "wake_up.py")
    # wake up the robot
    done = subprocess.run(["python2", wake_up_script, f"--ip=10.1.1.4"])
    # laser filter node
    # filter_node = Node(
    #    package="laser_filter",
    #    executable="laser_filter",
    #    name="filter_node",
    #    output="screen",
    # )
    return launch.LaunchDescription(
        [
            ip_declare,
            launch.actions.DeclareLaunchArgument(
                'nao_port', default_value="9559", description='Port to be used for the connection'
            ),
            launch.actions.DeclareLaunchArgument(
                'username', default_value="nao", description='Username for the connection'
            ),
            launch.actions.DeclareLaunchArgument(
                'password', default_value="no_password", description='Password for the connection'
            ),
            launch.actions.DeclareLaunchArgument(
                'network_interface', default_value="eth0", description='Network interface to be used'
            ),
            launch.actions.DeclareLaunchArgument(
                'namespace', default_value="naoqi_driver", description='Name of the namespace to be used'
            ),
            launch_ros.actions.Node(
                package='naoqi_driver',
                executable='naoqi_driver_node',
                name=[launch.substitutions.LaunchConfiguration('namespace')],
                parameters=[
                    {
                        'nao_ip': nao_ip,
                        'nao_port': launch.substitutions.LaunchConfiguration('nao_port'),
                        'password': launch.substitutions.LaunchConfiguration('password'),
                        'network_interface': launch.substitutions.LaunchConfiguration('network_interface'),
                    }
                ],
                output="screen",
            ),
            # filter_node,
        ]
    )
