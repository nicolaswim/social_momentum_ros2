#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
# from rclpy.clock import ClockType # Not strictly needed if we remove the problematic arg
import math
import numpy as np
from typing import List, Tuple, Optional

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from my_social_nav_interfaces.msg import HumanArray

try:
    from . import social_momentum as sm
    from . import geometry_utils as geo
except ImportError as e:
    import sys
    import os
    node_module_path = os.path.dirname(os.path.abspath(__file__))
    print(f"Error importing relative modules. Current path: {node_module_path}", file=sys.stderr)
    print(f"Python sys.path: {sys.path}", file=sys.stderr)
    print(f"Original ImportError: {e}", file=sys.stderr)
    raise ImportError(f"Cannot import .social_momentum or .geometry_utils from {node_module_path}. Check package structure and __init__.py.") from e

# Helper function to convert quaternion to yaw
def quaternion_to_yaw(orientation_q) -> float:
    qx, qy, qz, qw = orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

class SocialPlannerNode(Node):
    def __init__(self):
        super().__init__('social_planner_node')
        self.get_logger().info("Social Planner Node Initializing...")

        # Declare parameters with default values
        self.declare_parameter('social_momentum_weight', 0.1)
        self.declare_parameter('robot_fov_deg', 180.0)
        self.declare_parameter('robot_radius', 0.5)
        self.declare_parameter('human_radius', 0.5)
        self.declare_parameter('time_step', 0.1)
        self.declare_parameter('robot_max_speed', 1.0)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/mobile_base_controller/odom') 
        self.declare_parameter('humans_topic', '/humans')
        self.declare_parameter('cmd_vel_topic', '/mobile_base_controller/cmd_vel_unstamped') 
        self.declare_parameter('planning_frequency', 10.0)
        self.declare_parameter('auto_goal_x', math.nan)
        self.declare_parameter('auto_goal_y', math.nan)
        self.declare_parameter('goal_reached_threshold', 0.2)
        self.declare_parameter('initial_planner_delay_sec', 5.0) # Delay for planner to start

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
        auto_goal_x = self.get_parameter('auto_goal_x').value
        auto_goal_y = self.get_parameter('auto_goal_y').value
        self.goal_reached_threshold = self.get_parameter('goal_reached_threshold').value
        self.initial_planner_delay_sec = self.get_parameter('initial_planner_delay_sec').value


        self.get_logger().info(f"Parameters set: lambda_sm={self.lambda_sm}, robot_radius={self.robot_radius}, cmd_vel_topic={cmd_vel_topic}")
        self.get_logger().info(f"Goal reached threshold: {self.goal_reached_threshold} m")
        self.get_logger().info(f"Initial planner delay: {self.initial_planner_delay_sec} s")


        # State Variables
        self.current_robot_pose: Optional[np.ndarray] = None
        self.current_robot_orientation: Optional[float] = None
        self.current_robot_velocity: Optional[np.ndarray] = None
        self.current_goal: Optional[np.ndarray] = None
        self.human_data: List[Tuple[np.ndarray, np.ndarray]] = []

        # Auto goal initialization
        auto_goal_x_np = np.float64(auto_goal_x)
        auto_goal_y_np = np.float64(auto_goal_y)
        if not np.isnan(auto_goal_x_np) and not np.isnan(auto_goal_y_np):
            self.current_goal = np.array([auto_goal_x_np, auto_goal_y_np])
            self.get_logger().info(f"Using automatic goal: {self.current_goal}")
        else:
            self.get_logger().info(f"Waiting for goal on topic: {goal_topic}")

        self.robot_action_space = self._create_robot_action_space(self.robot_max_speed)
        self.get_logger().info(f"Created action space with {len(self.robot_action_space)} actions.")

        # QoS Profiles
        odom_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE 
        )
        self.get_logger().info(f"Using RELIABLE/VOLATILE QoS for {goal_topic} subscriber.")

        # Subscribers
        self.goal_subscriber = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, goal_qos)
        self.odom_subscriber = self.create_subscription(Odometry, odom_topic, self.odom_callback, odom_qos)
        self.human_subscriber = self.create_subscription(HumanArray, humans_topic, self.humans_callback, 10)
        
        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Timer
        self.timer_period = 1.0 / planning_frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self._initial_delay_passed = False
        self._logged_waiting_for_delay_once = False 

        self.get_logger().info("Social Planner Node Initialized Successfully.")

    def _create_robot_action_space(self, max_speed: float) -> List[np.ndarray]:
        actions = [np.array([0.0, 0.0])] # Stop action
        angles = np.linspace(-np.pi / 4, np.pi / 4, 7) # Angles from -45 to +45 degrees
        speeds = [max_speed * 0.5, max_speed]      # Half and full speed
        for speed in speeds:
            for angle in angles:
                # Action is [vx, vy] in robot's base_link frame
                action = np.array([speed * np.cos(angle), speed * np.sin(angle)])
                actions.append(action)
        return actions

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f"--- GOAL CALLBACK TRIGGERED --- msg header frame: {msg.header.frame_id}")
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.current_goal = np.array([goal_x, goal_y])
        self.get_logger().info(f"Received new goal: {self.current_goal}")

    def odom_callback(self, msg: Odometry):
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        self.current_robot_pose = np.array([pos_x, pos_y])
        self.current_robot_orientation = quaternion_to_yaw(msg.pose.pose.orientation)
        
        # Odometry twist is usually in the child_frame_id (base_link for Tiago)
        vel_x = msg.twist.twist.linear.x 
        vel_y = msg.twist.twist.linear.y 
        self.current_robot_velocity = np.array([vel_x, vel_y])
        # self.get_logger().debug(f"Odom: P={self.current_robot_pose}, Yaw={self.current_robot_orientation:.2f}, V_base={self.current_robot_velocity}", throttle_duration_sec=1.0)

    def humans_callback(self, msg: HumanArray):
        new_human_data = []
        for human_msg in msg.humans:
            pos = np.array([human_msg.position.x, human_msg.position.y])
            vel = np.array([human_msg.velocity.x, human_msg.velocity.y])
            new_human_data.append((pos, vel))
        self.human_data = new_human_data

    def timer_callback(self):
        # Initial delay logic
        if not self._initial_delay_passed:
            if self.get_clock().now().nanoseconds / 1e9 < self.initial_planner_delay_sec:
                if not self._logged_waiting_for_delay_once:
                    self.get_logger().info(f"Waiting for initial planner delay ({self.initial_planner_delay_sec}s)...")
                    self._logged_waiting_for_delay_once = True
                return # Wait for delay to pass
            self._initial_delay_passed = True
            self.get_logger().info("Initial planner delay passed. Planner active.")
        
        # Check for required data
        goal_needed = self.current_goal is None
        if self.current_robot_pose is None or \
           self.current_robot_orientation is None or \
           self.current_robot_velocity is None or \
           goal_needed:
            waiting_for = []
            if self.current_robot_pose is None: waiting_for.append("robot pose (odom)")
            if self.current_robot_orientation is None: waiting_for.append("robot orientation (odom)")
            if self.current_robot_velocity is None: waiting_for.append("robot velocity (odom)")
            if goal_needed: waiting_for.append("goal")
            
            # *** CORRECTED LOGGING CALL: Removed throttle_time_source_type ***
            self.get_logger().warn(f"Waiting for: {', '.join(waiting_for)}...", throttle_duration_sec=5.0) 
            
            stop_cmd = Twist() # Publish zero velocity if critical data is missing
            self.cmd_vel_publisher.publish(stop_cmd)
            return

        # Stop if goal is reached
        if self.current_goal is not None and self.current_robot_pose is not None:
            dist_to_goal = np.linalg.norm(self.current_robot_pose - self.current_goal)
            if dist_to_goal < self.goal_reached_threshold:
                self.get_logger().info(f"Goal reached! Distance: {dist_to_goal:.2f}m. Stopping robot.")
                stop_cmd = Twist() 
                self.cmd_vel_publisher.publish(stop_cmd)
                # self.current_goal = None # Optionally clear goal once reached
                return

        # Prepare inputs for the social momentum function
        robot_q = self.current_robot_pose
        robot_theta = self.current_robot_orientation
        current_robot_velocity_base = self.current_robot_velocity # This is from odom, in base_link frame
        robot_goal_q = self.current_goal
        
        all_humans_q = [data[0] for data in self.human_data]
        all_humans_v = [data[1] for data in self.human_data] # Assumed world frame from publisher

        action_space_current = list(self.robot_action_space)
        # Add robot's current velocity (in base_link frame) to potential actions if moving
        if np.linalg.norm(current_robot_velocity_base) > geo.EPSILON:
             action_space_current.append(current_robot_velocity_base)

        # Call the core social momentum algorithm
        selected_action_base = sm.select_social_momentum_action(
            robot_q=robot_q,
            robot_orientation_theta=robot_theta, 
            current_robot_velocity_base=current_robot_velocity_base, 
            robot_goal_q=robot_goal_q,
            all_humans_q=all_humans_q,
            all_humans_v=all_humans_v,
            robot_action_space_base=action_space_current, 
            lambda_sm=self.lambda_sm,
            time_step=self.time_step,
            robot_radius=self.robot_radius,
            human_radius=self.human_radius,
            fov_deg=self.fov_deg
        )
        
        # Publish the resulting command
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(selected_action_base[0])    # vx from base_link action
        turn_scale = 1.0 
        cmd_vel_msg.angular.z = float(-turn_scale * selected_action_base[1]) # vy from base_link action scaled to angular.z
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(f"Published cmd_vel: lin_x={cmd_vel_msg.linear.x:.2f}, ang_z={cmd_vel_msg.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SocialPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("Ctrl+C detected, shutting down node.")
    except Exception as e:
        if node:
             node.get_logger().error(f"Exception in node: {e}")
             import traceback
             node.get_logger().error(traceback.format_exc())
        else:
             print(f"Exception before node initialization: {e}", file=sys.stderr)
             import traceback
             traceback.print_exc()
    finally:
        if node and rclpy.ok() and hasattr(node, 'cmd_vel_publisher') and node.cmd_vel_publisher:
             stop_cmd = Twist()
             node.cmd_vel_publisher.publish(stop_cmd)
             node.get_logger().info("Published stop command before destroying node.")
        if node and rclpy.ok(): # Ensure node exists before destroying
             node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()
