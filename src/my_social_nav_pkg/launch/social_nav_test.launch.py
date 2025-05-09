import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory for my_social_nav_pkg (useful for other package files if needed)
    # my_social_nav_pkg_share_dir = get_package_share_directory('my_social_nav_pkg')
    
    # --- Try to get tiago_gazebo package directory ---
    # This is specific to TIAGo. If you are not using TIAGo, you'd use a generic gazebo_ros launch.
    tiago_gazebo_pkg_share_dir = ""
    tiago_launch_available = False
    try:
        tiago_gazebo_pkg_share_dir = get_package_share_directory('tiago_gazebo')
        tiago_launch_available = True
    except PackageNotFoundError:
        pass # tiago_launch_available will remain False

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', # Default to true as you use it with Gazebo
        description='Use simulation (Gazebo) clock if true'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty.world', 
        description='Gazebo world file name (e.g., empty.world, pal_office.world for TIAGo)'
    )

    # Arguments for fake_human_publisher
    scenario_mode_arg = DeclareLaunchArgument(
        'scenario_mode',
        default_value='random',
        choices=['head_on', 'random', 'teleop'],
        description='Scenario for fake human publisher'
    )

    num_random_humans_arg = DeclareLaunchArgument(
        'num_random_humans',
        default_value='3',
        description='Number of humans for random scenario'
    )

    # RViz configuration file argument
    # Point to your existing RViz configuration file in your home directory
    default_rviz_config_path = os.path.expanduser('~/.rviz2/socialmomentumdefault.rviz')
    
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Full path to the RViz configuration file to use'
    )
    
    # Argument to decide whether to launch Gazebo
    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='true', # Set to 'false' if you want to run Gazebo separately
        description='Whether to launch Gazebo simulation environment'
    )

    # --- Define Actions ---

    # Group for Gazebo related actions (only if launch_gazebo is true)
    gazebo_actions_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        actions=[
            LogInfo(
                condition=IfCondition(PythonExpression([f"not {tiago_launch_available}"])),
                msg="Package 'tiago_gazebo' not found. TIAGo Gazebo simulation will not be launched by this file. Ensure TIAGo simulation is running separately if needed, or install 'tiago_gazebo'."
            ),
            LogInfo(
                condition=IfCondition(PythonExpression([f"{tiago_launch_available}"])),
                msg=PythonExpression(["'Found tiago_gazebo package. Will attempt to launch TIAGo Gazebo with world: ', '", LaunchConfiguration('world_name'), "'"])
            ),
            # Include TIAGo Gazebo Launch (conditionally, if tiago_gazebo is found)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tiago_gazebo_pkg_share_dir, 'launch', 'tiago_gazebo.launch.py')
                ),
                condition=IfCondition(PythonExpression([f"{tiago_launch_available}"])),
                launch_arguments={
                    'is_public_sim': 'True',
                    'world_name': LaunchConfiguration('world_name'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    # Add other TIAGo specific arguments if needed
                }.items()
            )
            # If you wanted a generic Gazebo instead of TIAGo's, you would add it here:
            # (Requires gazebo_ros package to be installed)
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            #     ),
            #     condition=IfCondition(PythonExpression([f"not {tiago_launch_available}"])), # Example: launch if TIAGo not found
            #     launch_arguments={'world': LaunchConfiguration('world_name'), 
            #                       'verbose': 'true',
            #                       'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
            # ),
        ]
    )
    
    # Launch RViz2
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2', # Node name for RViz
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')], # Use the rviz_config argument
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}] # Pass use_sim_time
    )

    # Launch Social Planner Node
    start_social_planner_cmd = Node(
        package='my_social_nav_pkg',
        executable='social_planner_node',
        name='social_planner_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch Fake Human Publisher
    start_fake_human_publisher_cmd = Node(
        package='my_social_nav_pkg',
        executable='fake_human_publisher',
        name='fake_human_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'scenario_mode': LaunchConfiguration('scenario_mode')},
            {'num_random_humans': LaunchConfiguration('num_random_humans')},
        ]
    )

    # Conditionally Launch Teleop Keyboard for human control in 'teleop' scenario
    start_teleop_keyboard_cmd = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='human_teleop_keyboard', 
        output='screen',
        prefix='xterm -e', 
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('scenario_mode'), "' == 'teleop'"])),
        remappings=[('/cmd_vel', '/human_teleop_cmd_vel')] 
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_name_arg)
    ld.add_action(scenario_mode_arg)
    ld.add_action(num_random_humans_arg)
    ld.add_action(rviz_config_file_arg) # Add the rviz_config argument
    ld.add_action(launch_gazebo_arg)
    
    # Add actions to execute
    ld.add_action(gazebo_actions_group) # Gazebo (conditionally)
    ld.add_action(start_rviz_cmd)       # RViz
    ld.add_action(start_social_planner_cmd)
    ld.add_action(start_fake_human_publisher_cmd)
    ld.add_action(start_teleop_keyboard_cmd)

    return ld
