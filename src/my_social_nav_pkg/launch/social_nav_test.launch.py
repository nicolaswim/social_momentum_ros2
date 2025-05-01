import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    # Get package directories
    tiago_gazebo_pkg = get_package_share_directory('tiago_gazebo')
    my_social_nav_pkg_path = get_package_share_directory('my_social_nav_pkg') # Not strictly needed for Node action, but good practice

    # --- Declare Launch Arguments ---

    # Simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Gazebo world
    world_name = LaunchConfiguration('world_name', default='empty') # Use empty world
    # Human publisher scenario
    scenario_mode = LaunchConfiguration('scenario_mode', default='random') # 'head_on', 'random', 'teleop'
    # Number of humans for random mode
    num_random_humans = LaunchConfiguration('num_random_humans', default='3')

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')

    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='Gazebo world file name (e.g., empty, pal_office)')

    declare_scenario_mode_cmd = DeclareLaunchArgument(
        'scenario_mode',
        default_value='random',
        choices=['head_on', 'random', 'teleop'],
        description='Scenario for fake human publisher')

    declare_num_random_humans_cmd = DeclareLaunchArgument(
        'num_random_humans',
        default_value='3',
        description='Number of humans for random scenario')

    # --- Include Gazebo Launch ---
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tiago_gazebo_pkg, 'launch', 'tiago_gazebo.launch.py')
        ),
        launch_arguments={
            'is_public_sim': 'True',
            'world_name': world_name,
            'use_sim_time': use_sim_time # Pass sim time argument to Gazebo launch
        }.items()
    )

    # --- Launch Your Nodes ---
    start_social_planner_cmd = Node(
        package='my_social_nav_pkg',
        executable='social_planner_node',
        name='social_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] # Tell node to use sim time
    )

    start_fake_human_publisher_cmd = Node(
        package='my_social_nav_pkg',
        executable='fake_human_publisher',
        name='fake_human_publisher',
        output='screen',
        parameters=[ # Pass parameters to the human publisher
            {'use_sim_time': use_sim_time},
            {'scenario_mode': scenario_mode},
            {'num_random_humans': num_random_humans}
            # Add other parameters if needed (e.g., x_limits, y_limits)
            # {'x_limits': [-6.0, 6.0]},
            # {'y_limits': [-6.0, 6.0]}
        ]
    )

    # --- Conditionally Launch Teleop Keyboard ---
    start_teleop_keyboard_cmd = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='human_teleop_keyboard',
        output='screen',
        prefix='xterm -e', # Launch in a separate terminal window
        # Only launch if scenario_mode is 'teleop'
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('scenario_mode'), "' == 'teleop'"])),
        # Remap the output topic
        remappings=[('/cmd_vel', '/human_teleop_cmd_vel')]
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_scenario_mode_cmd)
    ld.add_action(declare_num_random_humans_cmd)

    # Add nodes and included launch files
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_social_planner_cmd)
    ld.add_action(start_fake_human_publisher_cmd)
    ld.add_action(start_teleop_keyboard_cmd) # Will only run if condition is met

    return ld