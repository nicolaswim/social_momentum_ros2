#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import random
import math
import time # For initial delay
import sys # For printing errors if imports fail
import os # For printing errors if imports fail

# Standard ROS messages
from geometry_msgs.msg import Point, Vector3, Twist, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Import our custom messages
try:
    from my_social_nav_interfaces.msg import HumanPoseVel, HumanArray
except ImportError:
    print("ERROR: Cannot import custom messages from my_social_nav_interfaces.", file=sys.stderr)
    print("Ensure the interface package is built and the current workspace is sourced.", file=sys.stderr)
    sys.exit(1)


class FakeHumanPublisher(Node):
    def __init__(self):
        super().__init__('fake_human_publisher')
        self.get_logger().info("--- FakeHumanPublisher Node Initializing ---") # <<< LOG

        # --- Parameters ---
        self.declare_parameter('scenario_mode', 'random') # 'head_on', 'random', 'teleop'
        self.declare_parameter('num_random_humans', 3)    # For 'random' mode
        self.declare_parameter('publish_frequency', 10.0) # Hz
        self.declare_parameter('human_max_speed', 1.5)    # Max speed for random humans
        self.declare_parameter('human_min_speed', 0.5)    # Min speed for random humans
        self.declare_parameter('world_frame_id', 'odom')  # Frame ID for published data
        self.declare_parameter('humans_topic', '/humans') # Topic to publish HumanArray on
        self.declare_parameter('markers_topic', '/human_markers') # Topic to publish MarkerArray on
        # Define world boundaries for spawning/movement (adjust based on Gazebo view)
        self.declare_parameter('x_limits', [-5.0, 5.0])
        self.declare_parameter('y_limits', [-5.0, 5.0])
        self.declare_parameter('teleop_cmd_topic', '/human_teleop_cmd_vel') # Topic for teleop input

        # Get parameters
        self.scenario = self.get_parameter('scenario_mode').value
        self.num_random = self.get_parameter('num_random_humans').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.max_speed = self.get_parameter('human_max_speed').value
        self.min_speed = self.get_parameter('human_min_speed').value
        self.frame_id = self.get_parameter('world_frame_id').value
        humans_topic = self.get_parameter('humans_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        self.x_lim = self.get_parameter('x_limits').value
        self.y_lim = self.get_parameter('y_limits').value
        teleop_cmd_topic = self.get_parameter('teleop_cmd_topic').value

        self.get_logger().info(f"Scenario: '{self.scenario}', Frame: '{self.frame_id}'") # <<< LOG

        # --- State ---
        self.humans = [] # List to store human states [ {id, pos, vel}, ... ]
        self.teleop_velocity = np.array([0.0, 0.0]) # Current velocity for teleop human

        # --- Initialization ---
        self.get_logger().info("Initializing humans...") # <<< LOG
        self._initialize_humans()
        self.get_logger().info(f"Humans initialized: {len(self.humans)} humans") # <<< LOG

        # --- Publisher ---
        self.publisher_ = self.create_publisher(HumanArray, humans_topic, 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, markers_topic, 10)
        self.get_logger().info("Publishers created.") # <<< LOG

        # --- Subscriber (for Teleop Mode) ---
        if self.scenario == 'teleop':
            self.teleop_sub = self.create_subscription(
                Twist,
                teleop_cmd_topic,
                self.teleop_cmd_callback,
                10)
            self.get_logger().info(f"Teleop subscriber created for {teleop_cmd_topic}") # <<< LOG
        else:
             self.teleop_sub = None

        # --- Timer ---
        self.timer_period = 1.0 / self.frequency
        # Add a small delay before starting the timer to allow other nodes (like planner) to start first
        time.sleep(1.0)
        self.timer = self.create_timer(self.timer_period, self.update_and_publish)
        self.get_logger().info(f"Timer created with period {self.timer_period}s.") # <<< LOG
        self.get_logger().info("--- FakeHumanPublisher Initialization Complete ---") # <<< LOG


    def _initialize_humans(self):
        """Initializes human positions and velocities based on the scenario."""
        self.humans = []
        if self.scenario == 'head_on':
            # Scenario 1: One human walking towards origin (approx robot start)
            start_y = self.y_lim[1] - 0.5 # Start near top edge
            start_x = 0.0               # Start in middle horizontally
            speed = random.uniform(self.min_speed, self.max_speed)
            human = {
                'id': 'human_0',
                'pos': np.array([start_x, start_y]),
                'vel': np.array([0.0, -speed]) # Moving straight down (towards Y=0)
            }
            self.humans.append(human)
            # Log moved to __init__ after calling this function

        elif self.scenario == 'random':
            # Scenario 2: N humans with random pos/vel
            for i in range(self.num_random):
                pos_x = random.uniform(self.x_lim[0], self.x_lim[1])
                pos_y = random.uniform(self.y_lim[0], self.y_lim[1])
                speed = random.uniform(self.min_speed, self.max_speed)
                angle = random.uniform(0, 2 * math.pi)
                vel_x = speed * math.cos(angle)
                vel_y = speed * math.sin(angle)
                human = {
                    'id': f'human_{i}',
                    'pos': np.array([pos_x, pos_y]),
                    'vel': np.array([vel_x, vel_y])
                }
                self.humans.append(human)
            # Log moved to __init__ after calling this function

        elif self.scenario == 'teleop':
             # Scenario 3: One human, starting near center, controlled by teleop
            start_x = 0.0
            start_y = (self.y_lim[0] + self.y_lim[1]) / 2.0 # Start in middle vertically
            human = {
                'id': 'human_teleop_0',
                'pos': np.array([start_x, start_y]),
                'vel': np.array([0.0, 0.0]) # Initially stationary
            }
            self.humans.append(human)
            # Log moved to __init__ after calling this function

        else:
            self.get_logger().error(f"Unknown scenario mode in _initialize_humans: {self.scenario}")


    def teleop_cmd_callback(self, msg: Twist):
         """Updates the target velocity for the teleop human."""
         self.get_logger().debug("Teleop callback received msg") # <<< OPTIONAL DEBUG LOG
         if self.scenario == 'teleop' and self.humans:
             # Use linear.x as vx, linear.y as vy
             self.teleop_velocity = np.array([msg.linear.x, msg.linear.y])
             self.humans[0]['vel'] = self.teleop_velocity


    def update_and_publish(self):
        """Updates human positions and publishes the HumanArray message AND MarkerArray."""
        self.get_logger().debug("--- update_and_publish called ---") # <<< DEBUG LOG
        dt = self.timer_period
        human_msgs = []
        marker_array_msg = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        human_id_counter = 0 # Use index for marker ID

        if not self.humans:
             self.get_logger().debug("No humans in list to update/publish.") # <<< DEBUG LOG
             # Optional: Publish empty marker array with DELETEALL action to clear RViz
             # delete_marker = Marker(header=Header(stamp=current_time, frame_id=self.frame_id), action=Marker.DELETEALL)
             # marker_array_msg.markers.append(delete_marker)
             # self.marker_publisher_.publish(marker_array_msg)
             return # Nothing else to do if list is empty

        for human in self.humans:
            # Update position based on velocity
            human['pos'] = human['pos'] + human['vel'] * dt

            # Simple boundary handling (wrap around for random, stop/reverse for head-on?)
            if self.scenario == 'random':
                if not (self.x_lim[0] < human['pos'][0] < self.x_lim[1]):
                    human['vel'][0] *= -1 # Reverse x velocity
                    human['pos'][0] = np.clip(human['pos'][0], self.x_lim[0]+0.1, self.x_lim[1]-0.1)
                if not (self.y_lim[0] < human['pos'][1] < self.y_lim[1]):
                    human['vel'][1] *= -1 # Reverse y velocity
                    human['pos'][1] = np.clip(human['pos'][1], self.y_lim[0]+0.1, self.y_lim[1]-0.1)
            # Add boundary handling for other modes if desired

            # --- Create HumanPoseVel message ---
            hpv_msg = HumanPoseVel()
            hpv_msg.human_id = human['id']
            hpv_msg.position.x = human['pos'][0]
            hpv_msg.position.y = human['pos'][1]
            hpv_msg.position.z = 0.0
            hpv_msg.velocity.x = human['vel'][0]
            hpv_msg.velocity.y = human['vel'][1]
            hpv_msg.velocity.z = 0.0
            human_msgs.append(hpv_msg)

            # --- Create Marker message for RViz (Cylinder) ---
            marker = Marker()
            marker.header.frame_id = self.frame_id # e.g., "odom"
            marker.header.stamp = current_time
            marker.ns = "humans" # Namespace for markers
            marker.id = human_id_counter # Use index for ID
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = human['pos'][0]
            marker.pose.position.y = human['pos'][1]
            marker.pose.position.z = 0.5 # Base of cylinder slightly off the ground
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # Default orientation
            marker.scale = Vector3(x=0.4, y=0.4, z=1.0) # Diameter 0.4m, Height 1.0m
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8) # Green, slightly transparent
            marker.lifetime = rclpy.duration.Duration(seconds=self.timer_period * 2.0).to_msg()
            marker_array_msg.markers.append(marker)
            human_id_counter += 1 # Increment for next marker ID

            # --- Optional: Velocity Arrow Marker ---
            vel_marker = Marker()
            vel_marker.header = marker.header # Copy header (frame, stamp)
            vel_marker.ns = "human_velocity"
            vel_marker.id = human_id_counter # Use different ID
            vel_marker.type = Marker.ARROW
            vel_marker.action = Marker.ADD

            start_point = Point(x=human['pos'][0], y=human['pos'][1], z=1.1) # Start arrow above cylinder
            vel_scale = 0.5 # Make arrow length proportional to speed (tune scale)
            # Only show arrow if velocity is significant
            if np.linalg.norm(human['vel']) > 0.1:
                 end_point = Point(
                     x=start_point.x + human['vel'][0] * vel_scale,
                     y=start_point.y + human['vel'][1] * vel_scale,
                     z=start_point.z # Keep arrow level for now
                 )
                 vel_marker.points = [start_point, end_point]
                 vel_marker.scale = Vector3(x=0.05, y=0.1, z=0.15) # Shaft diam, head diam, head len
                 vel_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8) # Red
                 vel_marker.lifetime = marker.lifetime
                 marker_array_msg.markers.append(vel_marker)
            else:
                 # If velocity is near zero, publish delete for previous arrow or set scale to zero
                 vel_marker.action = Marker.DELETE # Or set scale.x/y/z to tiny value if DELETE flickers
                 marker_array_msg.markers.append(vel_marker)

            human_id_counter += 1 # Increment again for next human


        # --- Publish Messages ---
        # Publish the HumanArray for the planner
        array_msg = HumanArray()
        array_msg.header = Header(stamp=current_time, frame_id=self.frame_id)
        array_msg.humans = human_msgs
        self.get_logger().debug(f"Publishing HumanArray with {len(array_msg.humans)} humans.") # <<< DEBUG LOG
        self.publisher_.publish(array_msg)

        # Publish the MarkerArray for RViz
        if marker_array_msg.markers:
            self.get_logger().debug(f"Publishing MarkerArray with {len(marker_array_msg.markers)} markers.") # <<< DEBUG LOG
            self.marker_publisher_.publish(marker_array_msg)
        else:
             self.get_logger().debug("No markers generated to publish.") # <<< DEBUG LOG


# --- Main Execution ---
def main(args=None):
    rclpy.init(args=args)
    node = None
    print(f"Starting fake_human_publisher node...") # Initial print before logging starts
    try:
        node = FakeHumanPublisher()
        print(f"Node created. Spinning...") # Debug print
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(" KeyboardInterrupt detected, shutting down node.")
    except Exception as e:
        if node:
             node.get_logger().error(f"Exception in node: {e}")
             import traceback
             node.get_logger().error(traceback.format_exc())
        else:
             print(f"Exception before node initialization: {e}")
             import traceback
             print(traceback.format_exc())
    finally:
        if node:
             print("Destroying node...")
             node.destroy_node()
        if rclpy.ok():
             print("Shutting down rclpy...")
             rclpy.shutdown()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()