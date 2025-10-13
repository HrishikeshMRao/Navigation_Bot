import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    This launch file is a hub for all the submodules to be launched in a sequence.
    Next, We launch the rsp launch file then we launch the gazebo world file.
    Third we launch the controllers rviz and the Executable files.
    """
    package_name = "navigation_bot"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    texture_name = "12"  # <--- CHANGE ME to change maze 0 to 1000

    world_path = os.path.join(
        get_package_share_directory(package_name), "worlds", "empty.world"
    )

    def Convoluter(context, *args, **kwargs):
        package_name = "navigation_bot"
        material_path = os.path.join(
            get_package_share_directory(package_name),
            "worlds",
            "materials",
        )

        return [
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(material_path, "scripts", "Convoluter.py"),
                    os.path.join(material_path, "textures", texture_name + ".png"),
                    os.path.join(material_path, "texture", texture_name + ".png"),
                ],
                output="screen",
            )
        ]

    def create_material(context=None):
        material_path = os.path.join(
            get_package_share_directory(package_name),
            "worlds",
            "materials",
            "scripts",
            "maze.material",
        )
        os.makedirs(os.path.dirname(material_path), exist_ok=True)
        with open(material_path, "w", encoding="utf-8") as f:
            f.write(
                f"""material Maze/diffuse
{{
    receive_shadows off
    technique
    {{
        pass
        {{
           lighting off            // disables light shading
           depth_write off         // prevents z-fighting with plane
           ambient 1 1 1           // full brightness
           diffuse 1 1 1           // no color tint
           emissive 1 1 1          // self-lit (glows)
           specular 0 0 0 0        // no shininess

            texture_unit
            {{
                texture {texture_name}.png
                filtering anisotropic
                max_anisotropy 16
            }}
        }}
    }}
}}"""
            )
        print(f"Generated material using texture: {texture_name}")

        # === Create world file that uses the material ===
        os.makedirs(os.path.dirname(world_path), exist_ok=True)

        return []

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world": world_path,
        }.items(),
    )

    # Run the spawner node from the gazebo_ros package.
    # The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "navigation_bot",
            "-x",
            "-4.7",  # Example offset in x direction
            "-y",
            "4.26",  # Example offset in y direction
            "-z",
            "0.0",  # Example offset in z direction
        ],
        output="screen",
    )

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

    image_processor = Node(
        package="navigation_bot",
        executable="ImageCapture",
        name="image_capture_node",
        parameters=[{"map": texture_name}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Flag to enable use_sim_time",
    )

    # WebcamPub = Node(
    #     package="navigation_bot",
    #     executable="webcam_publisher",
    #     name='webcam_capture_node',
    # )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            OpaqueFunction(function=Convoluter),
            OpaqueFunction(function=create_material),
            gazebo,
            spawn_entity,
            image_processor,
            rviz,
            diff_drive_spawner,
            joint_broad_spawner,
        ]
    )
