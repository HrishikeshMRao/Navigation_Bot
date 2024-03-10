import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch import LaunchDescription, conditions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='navigation_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'navigation_bot'],
                        output='screen')

    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    
    ImageProcessor = Node(
        package="navigation_bot",
        executable="ImageCapture",
        name='image_capture_node',
    )
    
    MotorDriver = Node(
        package="navigation_bot",
        executable="Motor_Driver_Node",
        name='Motor_Driver_Node',
    )
    
    # WebcamPub = Node(
    #     package="navigation_bot",
    #     executable="webcam_publisher",
    #     name='webcam_capture_node',
    # )
    
    joint_state_publisher_gui_node = Node( 
            package="joint_state_publisher_gui", 
            executable="joint_state_publisher_gui", 
            name="joint_state_publisher_gui", 
            condition=launch.conditions.IfCondition(LaunchConfiguration("gui")), 
    ) 
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        ImageProcessor,
        # MotorDriver,
        # WebcamPub,
        # diff_drive_spawner,
        # joint_broad_spawner,
        # joint_state_publisher_gui_node,
    ])