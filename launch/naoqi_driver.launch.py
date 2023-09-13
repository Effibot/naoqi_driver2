#! /usr/bin/python3

import launch
import launch_ros
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer
from launch import LaunchContext
from launch.events.process import ProcessStarted
import subprocess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("naoqi_driver")
    ip_declare = launch.actions.DeclareLaunchArgument(
        "nao_ip", default_value="127.0.0.1", description="Ip address of the robot"
    )
    nao_ip = launch.substitutions.LaunchConfiguration("nao_ip")
    # wake_up_script = os.path.join(pkg_dir, "launch", "wake_up.py")
    # wake up the robot
    # done = subprocess.run(["python2", wake_up_script, f"--ip=10.1.1.4"])
    # laser filter node
    # filter_node = Node(
    #    package="laser_filter",
    #    executable="laser_filter",
    #    name="filter_node",
    #    output="screen",
    # )

    comp_node = ComposableNodeContainer(
        name="container",
        namespace="/camera/depth/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzNode",
                name="point_cloud_xyz_node",
                remappings=[
                    ("image_rect", "/camera/depth/image_raw"),
                    ("camera_info", "/camera/depth/camera_info"),
                    ("points", "/camera/depth/points"),
                ],
            ),
        ],
        output="screen",
    )
    pc_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/camera/depth/points"), ("scan", "scan")],
        parameters=[
            {
                "target_frame": "base_link",
                "angle_increment": 0.017453292519943295,
                "angle_max": 0.523,
                "angle_min": -0.523,
                "inf_epsilon": 1.0,
                "max_height": 1.0,
                "min_height": 0.2,
                "queue_size": 12,
                "range_max": 7.0,
                "range_min": 0.0,
                "scan_time": 0.1,
                "transform_tolerance": 0.01,
                "use_inf": True,
                "use_sim_time": False,
            }
        ],
        name="pointcloud_to_laserscan",
    )
    
            
    depth = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [launch_ros.substitutions.FindPackageShare("pepper_nav"), "launch", "depthimage_scan.launch.py"]
            )
        )
    )
    laser_to_pointcloud = Node(
        package='pepper_nav',
        executable='laserconverter',
        name='laser_pc2',
        output='screen'
    )
    return launch.LaunchDescription(
        [
            ip_declare,
            launch.actions.DeclareLaunchArgument(
                "nao_port",
                default_value="9559",
                description="Port to be used for the connection",
            ),
            launch.actions.DeclareLaunchArgument(
                "username",
                default_value="nao",
                description="Username for the connection",
            ),
            launch.actions.DeclareLaunchArgument(
                "password",
                default_value="no_password",
                description="Password for the connection",
            ),
            launch.actions.DeclareLaunchArgument(
                "network_interface",
                default_value="eth0",
                description="Network interface to be used",
            ),
            launch.actions.DeclareLaunchArgument(
                "namespace",
                default_value="naoqi_driver",
                description="Name of the namespace to be used",
            ),
            launch_ros.actions.Node(
                package="naoqi_driver",
                executable="naoqi_driver_node",
                name=[launch.substitutions.LaunchConfiguration("namespace")],
                parameters=[
                    {
                        "nao_ip": nao_ip,
                        "nao_port": launch.substitutions.LaunchConfiguration(
                            "nao_port"
                        ),
                        "password": launch.substitutions.LaunchConfiguration(
                            "password"
                        ),
                        "network_interface": launch.substitutions.LaunchConfiguration(
                            "network_interface"
                        ),
                    }
                ],
                output="screen",
            ),
            depth,
            #laser_to_pointcloud
            #comp_node,
            #pc_to_laserscan_node,
        ]
    )
