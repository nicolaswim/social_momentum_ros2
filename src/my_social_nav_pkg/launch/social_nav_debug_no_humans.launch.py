import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- Try to get tiago_gazebo package directory ---
    tiago_gazebo_pkg_share_dir = ""
    tiago_launch_available = False
    try:
        tiago_gazebo_pkg_share_dir = get_package_share_directory('tiago_gazebo')
        tiago_launch_available = True
    except PackageNotFoundError:
        pass

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty.world', 
        description='Gazebo world file name'
    )

    # RViz configuration file argument
    default_rviz_config_path = os.path.expanduser('~/.rviz2/socialmomentumdefault.rviz')
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Full path to the RViz configuration file to use'
    )
    
    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='true',
        description='Whether to launch Gazebo simulation environment'
    )

    # --- Define Actions ---
    gazebo_actions_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        actions=[
            LogInfo(
                condition=IfCondition(PythonExpression([f"not {tiago_launch_available}"])),
                msg="Package 'tiago_gazebo' not found. TIAGo Gazebo will not be launched."
            ),
            LogInfo(
                condition=IfCondition(PythonExpression([f"{tiago_launch_available}"])),
                msg=PythonExpression(["'Found tiago_gazebo. Launching TIAGo Gazebo with world: ', '", LaunchConfiguration('world_name'), "'"])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tiago_gazebo_pkg_share_dir, 'launch', 'tiago_gazebo.launch.py')
                ),
                condition=IfCondition(PythonExpression([f"{tiago_launch_available}"])),
                launch_arguments={
                    'is_public_sim': 'True',
                    'world_name': LaunchConfiguration('world_name'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ]
    )
    
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    start_social_planner_cmd = Node(
        package='my_social_nav_pkg',
        executable='social_planner_node',
        name='social_planner_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)
    ld.add_action(world_name_arg)
    ld.add_action(rviz_config_file_arg)
    ld.add_action(launch_gazebo_arg)
    
    ld.add_action(gazebo_actions_group)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_social_planner_cmd)
    # Note: fake_human_publisher is NOT launched here

    return ld
