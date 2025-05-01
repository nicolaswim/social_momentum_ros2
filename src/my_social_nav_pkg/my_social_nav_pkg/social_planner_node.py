#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

import numpy as np
from typing import List, Tuple, Optional

# Import message types
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry # Common source for pose and velocity
from my_social_nav_interfaces.msg import HumanArray

# Import your custom modules using relative imports
try:
    from . import social_momentum as sm
    from . import geometry_utils as geo
except ImportError as e:
    # Improved error message for debugging
    import sys
    import os
    node_module_path = os.path.dirname(os.path.abspath(__file__))
    print(f"Error importing relative modules. Current path: {node_module_path}", file=sys.stderr)
    print(f"Python sys.path: {sys.path}", file=sys.stderr)
    print(f"Original ImportError: {e}", file=sys.stderr)
    raise ImportError(f"Cannot import .social_momentum or .geometry_utils from {node_module_path}. Check package structure and __init__.py.") from e
class SocialPlannerNode(Node):
    """
    ROS 2 Node to implement the Social Momentum navigation approach.
    """
    def __init__(self):
        super().__init__('social_planner_node')

        self.get_logger().info("Social Planner Node Initializing...")

        # --- Parameters ---
        self.declare_parameter('social_momentum_weight', 0.1) # Default from main_sim.py
        self.declare_parameter('robot_fov_deg', 180.0)
        self.declare_parameter('robot_radius', 0.5) # Default from environment.py
        self.declare_parameter('human_radius', 0.5) # Default from environment.py
        self.declare_parameter('time_step', 0.1) # Planning prediction time step
        self.declare_parameter('robot_max_speed', 1.0) # Default from environment.py
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/mobile_base_controller/odom') # Correct topic for Tiago sim
        self.declare_parameter('humans_topic', '/humans')
        self.declare_parameter('cmd_vel_topic', '/mobile_base_controller/cmd_vel_unstamped') # Common for Tiago
        self.declare_parameter('planning_frequency', 10.0) # Hz (e.g., run planner 10 times/sec)

        # Get parameter values
        self.lambda_sm = self.get_parameter('social_momentum_weight').value
        self.fov_deg = self.get_parameter('robot_fov_deg').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.human_radius = self.get_parameter('human_radius').value
        self.time_step = self.get_parameter('time_step').value
        self.robot_max_speed = self.get_parameter('robot_max_speed').value
        goal_topic = self.get_parameter('goal_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        humans_topic = self.get_parameter('humans_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        planning_frequency = self.get_parameter('planning_frequency').value

        self.get_logger().info(f"Parameters set:")
        self.get_logger().info(f"  lambda_sm: {self.lambda_sm}")
        self.get_logger().info(f"  robot_radius: {self.robot_radius}")
        self.get_logger().info(f"  human_radius: {self.human_radius}")
        # ... log other important parameters ...
        self.get_logger().info(f"  cmd_vel_topic: {cmd_vel_topic}")

        # --- State Variables ---
        self.current_robot_pose: Optional[np.ndarray] = None # Store as [x, y, theta] or just [x, y]
        self.current_robot_velocity: Optional[np.ndarray] = None # Store as [vx, vy]
        self.current_goal: Optional[np.ndarray] = None # Store as [x, y]
        self.human_data: List[Tuple[np.ndarray, np.ndarray]] = [] # List of (pos [x,y], vel [vx,vy]) tuples

        # --- Action Space ---
        # This part IS taken from main_sim.py's create_robot_action_space function
        self.robot_action_space = self._create_robot_action_space(self.robot_max_speed)
        self.get_logger().info(f"Created action space with {len(self.robot_action_space)} actions.")

        # --- QoS Profiles ---
        # Consistent QoS for Odometry and TF is important
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Or RELIABLE if available
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # QoS for goal (often published infrequently)
        goal_qos = QoSProfile(
             reliability=ReliabilityPolicy.RELIABLE,
             history=HistoryPolicy.KEEP_LAST,
             depth=1,
             durability=DurabilityPolicy.TRANSIENT_LOCAL # Keep last goal
        )
         # TODO: Define QoS for human data based on its source

        # --- Subscribers ---
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            goal_qos)

        self.odom_subscriber = self.create_subscription(
            Odometry, # Using Odometry which contains both pose and twist
            odom_topic,
            self.odom_callback,
            odom_qos)

        # Create subscriber for human data
        self.human_subscriber = self.create_subscription(
            HumanArray,
            humans_topic,
            self.humans_callback,
            10 # Or appropriate QoS
        )

        # --- Publisher ---
        self.cmd_vel_publisher = self.create_publisher(
            Twist, # Publishing Twist commands
            cmd_vel_topic,
            10) # QoS depth

        # --- Timer ---
        self.timer_period = 1.0 / planning_frequency # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Social Planner Node Initialized Successfully.")

    # --- Action Space Creation (from main_sim.py) ---
    def _create_robot_action_space(self, max_speed: float) -> List[np.ndarray]:
        """Creates the discrete set of possible velocity actions for the robot."""
        actions = [np.array([0.0, 0.0])] # Stop action
        # Reduced angles for potentially smoother control
        angles = np.linspace(-np.pi / 6, np.pi / 6, 5) # e.g., -30 to +30 degrees
        speeds = [max_speed * 0.5, max_speed] # Half speed and full speed

        # Assuming robot forward is +X in base_link frame for cmd_vel
        # If Tiago's forward is different, adjust sin/cos accordingly
        for speed in speeds:
            for angle in angles:
                # Twist message: linear.x, angular.z
                # If we map actions directly to vx, vy:
                # action = np.array([speed * np.cos(angle), speed * np.sin(angle)]) # Forward = +X
                # For Twist (linear.x, angular.z), need conversion or different action space
                # Let's keep the vx, vy action space for the *planner* for now
                # The planner output will be converted to Twist later if needed,
                # OR we modify the planner to output Twist directly.
                # Sticking to vx, vy for now based on social_momentum input needs:
                action = np.array([speed * np.cos(angle), speed * np.sin(angle)])
                actions.append(action)
        return actions

    # --- Subscriber Callbacks ---
    def goal_callback(self, msg: PoseStamped):
        """Stores the received goal pose."""
        # Extracting x, y from the PoseStamped message
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.current_goal = np.array([goal_x, goal_y])
        self.get_logger().info(f"Received new goal: {self.current_goal}")

    def odom_callback(self, msg: Odometry):
        """Updates current robot pose and velocity from odometry."""
        # Extract 2D pose (x, y)
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        self.current_robot_pose = np.array([pos_x, pos_y])

        # Extract 2D velocity (vx, vy) - usually in the robot's base_link frame
        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y # Often zero for non-holonomic robots
        # Note: Odometry Twist is often in base_link frame.
        # Your planner might need velocity in the map/odom frame.
        # This requires a TF transformation (more advanced).
        # For now, assuming planner can use base_link velocity or adapt.
        self.current_robot_velocity = np.array([vel_x, vel_y])

    def humans_callback(self, msg: HumanArray): # Use HumanArray type hint
        """Processes and stores human data from the HumanArray message."""
        # Check if the message frame_id matches what your planner expects (e.g., 'odom')
        # Optional: Add TF lookup here if frames don't match. For now, assume they do.
        # if msg.header.frame_id != 'odom':
        #     self.get_logger().warn(f"Received human data in frame '{msg.header.frame_id}', expected 'odom'.", throttle_duration_sec=10)
        #     # Consider adding TF transformation here
        #     return

        new_human_data = []
        for human_msg in msg.humans:
            # Extract position (using x, y)
            pos = np.array([human_msg.position.x, human_msg.position.y])
            # Extract velocity (using x, y)
            vel = np.array([human_msg.velocity.x, human_msg.velocity.y])
            # Append as a tuple (position_array, velocity_array)
            new_human_data.append((pos, vel))

        # Atomically update the shared data structure
        self.human_data = new_human_data
        # Optional debug log:
        # self.get_logger().debug(f"Updated human data: {len(self.human_data)} humans.")


    # --- Main Planning Logic (Timer Callback) ---
    def timer_callback(self):
        """Periodic function to run the social planner."""
        # Check if we have all necessary information
        if self.current_robot_pose is None or \
           self.current_robot_velocity is None or \
           self.current_goal is None:
            # TODO: Add check for human data availability
            self.get_logger().warn("Waiting for robot pose, velocity, goal, or human data...", throttle_duration_sec=5)
            # Optionally publish zero velocity to stop robot if data is missing
            # stop_cmd = Twist()
            # self.cmd_vel_publisher.publish(stop_cmd)
            return

        # Prepare inputs for the social momentum function
        robot_q = self.current_robot_pose
        current_robot_velocity = self.current_robot_velocity
        robot_goal_q = self.current_goal

        # TODO: Get human positions and velocities from self.human_data
        all_humans_q = [data[0] for data in self.human_data]
        all_humans_v = [data[1] for data in self.human_data]

        # Add robot's current velocity to potential actions if moving
        # (Logic snippet from main_sim.py step method)
        action_space_current = list(self.robot_action_space)
        if np.linalg.norm(current_robot_velocity) > geo.EPSILON:
             # Ensure current velocity format matches action space format (vx, vy)
             action_space_current.append(current_robot_velocity)


        # --- Call the Core Social Momentum Algorithm ---
        # This is the main call, using the function from your imported module
        selected_action = sm.select_social_momentum_action(
            robot_q=robot_q,
            current_robot_velocity=current_robot_velocity,
            robot_goal_q=robot_goal_q,
            all_humans_q=all_humans_q,
            all_humans_v=all_humans_v,
            robot_action_space=action_space_current,
            lambda_sm=self.lambda_sm,
            time_step=self.time_step,
            robot_radius=self.robot_radius,
            human_radius=self.human_radius,
            fov_deg=self.fov_deg
        )

        # --- Publish the Resulting Command ---
        cmd_vel_msg = Twist()
        # The selected_action is likely [vx, vy] based on your action space.
        # We need to map this to Twist's linear.x and angular.z for Tiago.
        # Simple mapping (assuming action[0] is desired forward speed):
        cmd_vel_msg.linear.x = selected_action[0]
        # Simple mapping (assuming action[1] is desired sideways speed, map to rotation):
        # This mapping might need significant tuning or a proper controller.
        # A basic proportional control for turning based on vy:
        turn_scale = 1.0 # Tune this factor
        cmd_vel_msg.angular.z = -turn_scale * selected_action[1] # Negative if y maps to left turn

        # Alternative: If planner outputs Twist directly, just assign fields.
        # cmd_vel_msg.linear.x = selected_action.linear.x
        # cmd_vel_msg.angular.z = selected_action.angular.z

        # Ensure speed limits are respected (optional, controller might handle this)
        # cmd_vel_msg.linear.x = np.clip(cmd_vel_msg.linear.x, -self.robot_max_speed, self.robot_max_speed)
        # Add angular velocity limits if needed

        self.cmd_vel_publisher.publish(cmd_vel_msg)
        # self.get_logger().info(f"Published cmd_vel: lin_x={cmd_vel_msg.linear.x:.2f}, ang_z={cmd_vel_msg.angular.z:.2f}")


def destroy_node(self):
    # Clean up resources (e.g., stop timers) if needed, although super().destroy_node() handles basics
    self.timer.cancel()
    super().destroy_node()

# --- Main Execution ---
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SocialPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(" Ctrl+C detected, shutting down node.")
    except Exception as e:
        if node:
             node.get_logger().error(f"Exception in node: {e}")
             import traceback
             node.get_logger().error(traceback.format_exc())
        else:
             print(f"Exception before node initialization: {e}")
    finally:
        if node:
             node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()

if __name__ == '__main__':
    main()