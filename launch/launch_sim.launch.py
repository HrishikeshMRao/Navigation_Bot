import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='navigation_bot' #<--- CHANGE ME
    default_model_path = "/home/fiend/Navigation_Bot/dev_ws/src/description/robot.urdf.xacro"

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'),'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'navigation_bot',
                                   '-x', '-5.0',  # Example offset in x direction
                                   '-y', '4.3',   # Example offset in y direction
                                   '-z', '0.0',   # Example offset in z direction
                                   ],
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
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(get_package_share_directory(package_name), 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
    )
    
    Rviz = Node(
       package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    Slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]), 
                launch_arguments={'params_file':'/home/fiend/Navigation_Bot/dev_ws/src/config/mapper_params_online_async.yaml','use_sim_time':'true'}.items()
            )
    
    Nav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')]), 
            )

    DeclareLaunchArgument(name='use_sim_time', 
                        default_value='true',
                        description='Flag to enable use_sim_time')
    
    # WebcamPub = Node(
    #     package="navigation_bot",
    #     executable="webcam_publisher",
    #     name='webcam_capture_node',
    # )
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        ImageProcessor,
        # robot_localization_node,
        # joint_state_publisher_node,
        Rviz,
        # Slam,
        # Nav,
        # MotorDriver,
        # WebcamPub,
        diff_drive_spawner,
        joint_broad_spawner,
    ])