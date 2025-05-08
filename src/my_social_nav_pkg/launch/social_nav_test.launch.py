# # # import os
# # # from ament_index_python.packages import get_package_share_directory
# # # from launch import LaunchDescription
# # # from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# # # from launch.launch_description_sources import PythonLaunchDescriptionSource
# # # from launch.substitutions import LaunchConfiguration, PythonExpression
# # # from launch.conditions import IfCondition
# # # from launch_ros.actions import Node

# # # def generate_launch_description():

# # #     # Get package directories
# # #     tiago_gazebo_pkg = get_package_share_directory('tiago_gazebo')
# # #     my_social_nav_pkg_path = get_package_share_directory('my_social_nav_pkg') # Not strictly needed for Node action, but good practice

# # #     # --- Declare Launch Arguments ---

# # #     # Simulation time
# # #     use_sim_time = LaunchConfiguration(m'use_sim_time', default='true')
# # #     # Gazebo world
# # #     world_name = LaunchConfiguration('world_name', default='empty') # Use empty world
# # #     # Human publisher scenario
# # #     scenario_mode = LaunchConfiguration('scenario_mode', default='random') # 'head_on', 'random', 'teleop'
# # #     # Number of humans for random mode
# # #     num_random_humans = LaunchConfiguration('num_random_huans', default='3')

# # #     # --- Declare Launch Arguments ---
# # #     declare_use_sim_time_cmd = DeclareLaunchArgument(
# # #         'use_sim_time',
# # #         default_value='true',
# # #         description='Use simulation clock')

# # #     declare_world_name_cmd = DeclareLaunchArgument(
# # #         'world_name',
# # #         default_value='empty',
# # #         description='Gazebo world file name (e.g., empty, pal_office)')

# # #     declare_scenario_mode_cmd = DeclareLaunchArgument(
# # #         'scenario_mode',
# # #         default_value='random',
# # #         choices=['head_on', 'random', 'teleop'],
# # #         description='Scenario for fake human publisher')

# # #     declare_num_random_humans_cmd = DeclareLaunchArgument(
# # #         'num_random_humans',
# # #         default_value='3',
# # #         description='Number of humans for random scenario')

# # #     # --- Include Gazebo Launch ---
# # #     start_gazebo_cmd = IncludeLaunchDescription(
# # #         PythonLaunchDescriptionSource(
# # #             os.path.join(tiago_gazebo_pkg, 'launch', 'tiago_gazebo.launch.py')
# # #         ),
# # #         launch_arguments={
# # #             'is_public_sim': 'True',
# # #             'world_name': world_name,
# # #             'use_sim_time': use_sim_time # Pass sim time argument to Gazebo launch
# # #         }.items()
# # #     )

# # #     # --- Launch Your Nodes ---
# # #     start_social_planner_cmd = Node(
# # #         package='my_social_nav_pkg',
# # #         executable='social_planner_node',
# # #         name='social_planner_node',
# # #         output='screen',
# # #         parameters=[{'use_sim_time': use_sim_time}] # Tell node to use sim time
# # #     )

# # #     start_fake_human_publisher_cmd = Node(
# # #         package='my_social_nav_pkg',
# # #         executable='fake_human_publisher',
# # #         name='fake_human_publisher',
# # #         output='screen',
# # #         parameters=[ # Pass parameters to the human publisher
# # #             {'use_sim_time': use_sim_time},
# # #             {'scenario_mode': scenario_mode},
# # #             {'num_random_humans': num_random_humans}
# # #             # Add other parameters if needed (e.g., x_limits, y_limits)
# # #             # {'x_limits': [-6.0, 6.0]},
# # #             # {'y_limits': [-6.0, 6.0]}
# # #         ]
# # #     )

# # #     # --- Conditionally Launch Teleop Keyboard ---
# # #     start_teleop_keyboard_cmd = Node(
# # #         package='teleop_twist_keyboard',
# # #         executable='teleop_twist_keyboard',
# # #         name='human_teleop_keyboard',
# # #         output='screen',
# # #         prefix='xterm -e', # Launch in a separate terminal window
# # #         # Only launch if scenario_mode is 'teleop'
# # #         condition=IfCondition(PythonExpression(["'", LaunchConfiguration('scenario_mode'), "' == 'teleop'"])),
# # #         # Remap the output topic
# # #         remappings=[('/cmd_vel', '/human_teleop_cmd_vel')]
# # #     )

# # #     # --- Create Launch Description ---
# # #     ld = LaunchDescription()

# # #     # Add declared arguments
# # #     ld.add_action(declare_use_sim_time_cmd)
# # #     ld.add_action(declare_world_name_cmd)
# # #     ld.add_action(declare_scenario_mode_cmd)
# # #     ld.add_action(declare_num_random_humans_cmd)

# # #     # Add nodes and included launch files
# # #     ld.add_action(start_gazebo_cmd)
# # #     ld.add_action(start_social_planner_cmd)
# # #     ld.add_action(start_fake_human_publisher_cmd)
# # #     ld.add_action(start_teleop_keyboard_cmd) # Will only run if condition is met

# # #     return ld

# # import os
# # import math
# # from ament_index_python.packages import get_package_share_directory
# # from launch import LaunchDescription
# # from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
# # from launch.launch_description_sources import PythonLaunchDescriptionSource
# # from launch.substitutions import LaunchConfiguration, PythonExpression
# # from launch.conditions import IfCondition
# # from launch_ros.actions import Node

# # # Define goal coordinates (adjust Y as needed based on your empty world size)
# # AUTO_GOAL_X = 0.0
# # AUTO_GOAL_Y = 10.0 # Example: Goal 10 meters forward

# # def launch_setup(context, *args, **kwargs):
# #     # Get launch arguments
# #     test_scenario = LaunchConfiguration('test_scenario').perform(context)
# #     num_random_humans_str = LaunchConfiguration('num_random_humans').perform(context)
# #     use_sim_time = LaunchConfiguration('use_sim_time') # Keep as LaunchConfiguration object

# #     # Prepare parameters based on scenario
# #     planner_params = [{'use_sim_time': use_sim_time}]
# #     publisher_params = [{'use_sim_time': use_sim_time}]
# #     human_scenario_mode = 'random' # Default for manual
# #     nodes_to_start = []
# #     start_teleop = False

# #     if test_scenario == 'head_on':
# #         human_scenario_mode = 'head_on'
# #         planner_params.append({'auto_goal_x': AUTO_GOAL_X})
# #         planner_params.append({'auto_goal_y': AUTO_GOAL_Y})
# #         publisher_params.append({'scenario_mode': human_scenario_mode})

# #     elif test_scenario == 'random':
# #         human_scenario_mode = 'random'
# #         planner_params.append({'auto_goal_x': AUTO_GOAL_X})
# #         planner_params.append({'auto_goal_y': AUTO_GOAL_Y})
# #         publisher_params.append({'scenario_mode': human_scenario_mode})
# #         publisher_params.append({'num_random_humans': int(num_random_humans_str)}) # Convert to int

# #     elif test_scenario == 'teleop':
# #         human_scenario_mode = 'teleop'
# #         planner_params.append({'auto_goal_x': AUTO_GOAL_X})
# #         planner_params.append({'auto_goal_y': AUTO_GOAL_Y})
# #         publisher_params.append({'scenario_mode': human_scenario_mode})
# #         start_teleop = True

# #     elif test_scenario == 'manual_goal':
# #         # Use default 'random' for humans, planner waits for topic
# #         human_scenario_mode = 'random' # Or make this configurable too
# #         publisher_params.append({'scenario_mode': human_scenario_mode})
# #         publisher_params.append({'num_random_humans': int(num_random_humans_str)})
# #         # Don't add auto_goal params to planner
# #         pass
# #     else:
# #         # Handle invalid scenario? For now, defaults to manual
# #         publisher_params.append({'scenario_mode': 'random'})
# #         publisher_params.append({'num_random_humans': int(num_random_humans_str)})


# #     # --- Define Nodes ---
# #     start_social_planner_cmd = Node(
# #         package='my_social_nav_pkg',
# #         executable='social_planner_node',
# #         name='social_planner_node',
# #         output='screen',
# #         parameters=planner_params
# #     )
# #     nodes_to_start.append(start_social_planner_cmd)

# #     start_fake_human_publisher_cmd = Node(
# #         package='my_social_nav_pkg',
# #         executable='fake_human_publisher',
# #         name='fake_human_publisher',
# #         output='screen',
# #         parameters=publisher_params
# #     )
# #     nodes_to_start.append(start_fake_human_publisher_cmd)

# #     # Conditionally start teleop
# #     if start_teleop:
# #         start_teleop_keyboard_cmd = Node(
# #             package='teleop_twist_keyboard',
# #             executable='teleop_twist_keyboard',
# #             name='human_teleop_keyboard',
# #             output='screen',
# #             prefix='xterm -e',
# #             remappings=[('/cmd_vel', '/human_teleop_cmd_vel')]
# #             # No condition needed here as we check start_teleop flag
# #         )
# #         nodes_to_start.append(start_teleop_keyboard_cmd)

# #     return nodes_to_start


# # def generate_launch_description():

# #     # Get package directories
# #     tiago_gazebo_pkg = get_package_share_directory('tiago_gazebo')

# #     # --- Declare Launch Arguments ---
# #     declare_use_sim_time_cmd = DeclareLaunchArgument(
# #         'use_sim_time',
# #         default_value='true',
# #         description='Use simulation clock')

# #     declare_world_name_cmd = DeclareLaunchArgument(
# #         'world_name',
# #         default_value='empty',
# #         description='Gazebo world file name')

# #     declare_test_scenario_cmd = DeclareLaunchArgument(
# #         'test_scenario',
# #         default_value='manual_goal', # Default to needing RViz goal
# #         choices=['head_on', 'random', 'teleop', 'manual_goal'],
# #         description='Select test scenario')

# #     declare_num_random_humans_cmd = DeclareLaunchArgument(
# #         'num_random_humans',
# #         default_value='3',
# #         description='Number of humans for random scenario')

# #     # --- Include Gazebo Launch ---
# #     start_gazebo_cmd = IncludeLaunchDescription(
# #         PythonLaunchDescriptionSource(
# #             os.path.join(tiago_gazebo_pkg, 'launch', 'tiago_gazebo.launch.py')
# #         ),
# #         launch_arguments={
# #             'is_public_sim': 'True', # Use capitalized string
# #             'world_name': LaunchConfiguration('world_name'),
# #             'use_sim_time': LaunchConfiguration('use_sim_time')
# #         }.items()
# #     )

# #     # --- Create Launch Description ---
# #     ld = LaunchDescription()

# #     # Add declared arguments first
# #     ld.add_action(declare_use_sim_time_cmd)
# #     ld.add_action(declare_world_name_cmd)
# #     ld.add_action(declare_test_scenario_cmd)
# #     ld.add_action(declare_num_random_humans_cmd)

# #     # Add Gazebo launch
# #     ld.add_action(start_gazebo_cmd)

# #     # Add OpaqueFunction to process arguments and define nodes
# #     ld.add_action(OpaqueFunction(function=launch_setup))

# #     return ld

# import os
# import math # Not strictly needed now but keep for potential future use
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PythonExpression
# from launch.conditions import IfCondition
# from launch_ros.actions import Node

# def generate_launch_description():

#     # Get package directories
#     tiago_gazebo_pkg = get_package_share_directory('tiago_gazebo')

#     # --- Declare Launch Arguments ---
#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='true',
#         description='Use simulation clock')

#     declare_world_name_cmd = DeclareLaunchArgument(
#         'world_name',
#         default_value='empty',
#         description='Gazebo world file name')

#     # Arguments directly for fake_human_publisher
#     declare_scenario_mode_cmd = DeclareLaunchArgument(
#         'scenario_mode',
#         default_value='random', # Default scenario
#         choices=['head_on', 'random', 'teleop'],
#         description='Scenario for fake human publisher')

#     declare_num_random_humans_cmd = DeclareLaunchArgument(
#         'num_random_humans',
#         default_value='3',
#         description='Number of humans for random scenario')

#     declare_start_delay_cmd = DeclareLaunchArgument(
#         'start_delay',
#         default_value='6.0', # Default start delay
#         description='Delay in seconds before humans start moving')

#     # --- Nodes and Includes ---

#     # Include Gazebo Launch
#     start_gazebo_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(tiago_gazebo_pkg, 'launch', 'tiago_gazebo.launch.py')
#         ),
#         launch_arguments={
#             'is_public_sim': 'True', # Use capitalized string
#             'world_name': LaunchConfiguration('world_name'),
#             'use_sim_time': LaunchConfiguration('use_sim_time')
#         }.items()
#     )

#     # Launch Social Planner Node (reacts to /goal_pose)
#     start_social_planner_cmd = Node(
#         package='my_social_nav_pkg',
#         executable='social_planner_node',
#         name='social_planner_node',
#         output='screen',
#         parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
#         # No auto_goal parameters passed
#     )

#     # Launch Fake Human Publisher
#     start_fake_human_publisher_cmd = Node(
#         package='my_social_nav_pkg',
#         executable='fake_human_publisher',
#         name='fake_human_publisher',
#         output='screen',
#         parameters=[ # Pass parameters directly using LaunchConfiguration
#             {'use_sim_time': LaunchConfiguration('use_sim_time')},
#             {'scenario_mode': LaunchConfiguration('scenario_mode')},
#             {'num_random_humans': LaunchConfiguration('num_random_humans')},
#             {'start_delay': LaunchConfiguration('start_delay')}
#             # Add world limits if you want them configurable via launch
#             # {'x_limits': [-6.0, 6.0]},
#             # {'y_limits': [-6.0, 10.0]}
#         ]
#     )

#     # Conditionally Launch Teleop Keyboard based on scenario_mode argument
#     start_teleop_keyboard_cmd = Node(
#         package='teleop_twist_keyboard',
#         executable='teleop_twist_keyboard',
#         name='human_teleop_keyboard',
#         output='screen',
#         prefix='xterm -e', # Launch in a separate terminal window
#         condition=IfCondition(PythonExpression(["'", LaunchConfiguration('scenario_mode'), "' == 'teleop'"])),
#         remappings=[('/cmd_vel', '/human_teleop_cmd_vel')]
#     )

#     # --- Create Launch Description ---
#     ld = LaunchDescription()

#     # Add declared arguments
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(declare_world_name_cmd)
#     ld.add_action(declare_scenario_mode_cmd)
#     ld.add_action(declare_num_random_humans_cmd)
#     ld.add_action(declare_start_delay_cmd)

#     # Add nodes and included launch files
#     ld.add_action(start_gazebo_cmd)
#     ld.add_action(start_social_planner_cmd)
#     ld.add_action(start_fake_human_publisher_cmd)
#     ld.add_action(start_teleop_keyboard_cmd)

#     return ld

import os
# Removed math import as it's not needed here anymore
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

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')

    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='Gazebo world file name')

    # Arguments directly for fake_human_publisher
    declare_scenario_mode_cmd = DeclareLaunchArgument(
        'scenario_mode',
        default_value='random', # Default scenario
        choices=['head_on', 'random', 'teleop'],
        description='Scenario for fake human publisher')

    declare_num_random_humans_cmd = DeclareLaunchArgument(
        'num_random_humans',
        default_value='3',
        description='Number of humans for random scenario')

    declare_start_delay_cmd = DeclareLaunchArgument(
        'start_delay',
        default_value='6.0', # Default start delay
        description='Delay in seconds before humans start moving')

    # --- Nodes and Includes ---

    # Include Gazebo Launch
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tiago_gazebo_pkg, 'launch', 'tiago_gazebo.launch.py')
        ),
        launch_arguments={
            'is_public_sim': 'True',
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Launch Social Planner Node (reacts to /goal_pose)
    start_social_planner_cmd = Node(
        package='my_social_nav_pkg',
        executable='social_planner_node',
        name='social_planner_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        # No auto_goal parameters passed
    )

    # Launch Fake Human Publisher
    start_fake_human_publisher_cmd = Node(
        package='my_social_nav_pkg',
        executable='fake_human_publisher',
        name='fake_human_publisher',
        output='screen',
        parameters=[ # Pass parameters directly using LaunchConfiguration
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'scenario_mode': LaunchConfiguration('scenario_mode')},
            {'num_random_humans': LaunchConfiguration('num_random_humans')},
            {'start_delay': LaunchConfiguration('start_delay')}
        ]
    )

    # Conditionally Launch Teleop Keyboard based on scenario_mode argument
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
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_scenario_mode_cmd)
    ld.add_action(declare_num_random_humans_cmd)
    ld.add_action(declare_start_delay_cmd)

    # Add nodes and included launch files
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_social_planner_cmd)
    ld.add_action(start_fake_human_publisher_cmd)
    ld.add_action(start_teleop_keyboard_cmd)

    return ld