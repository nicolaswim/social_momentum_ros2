# social_momentum.py
"""
Module implementing the core Social Momentum algorithm for robot action selection.
"""
import numpy as np
from typing import List, Tuple, Optional
import math # For sin/cos

# Use geometry utils for calculations
from . import geometry_utils as geo

# --- Algorithm Constants ---
DEFAULT_FOV_DEG = 180         # Field of view for reactive agents (degrees)


def _transform_base_to_world_velocity(base_velocity: np.ndarray, theta: float) -> np.ndarray:
    """Transforms a velocity from base_link frame to world frame."""
    vx_base, vy_base = base_velocity[0], base_velocity[1]
    vx_world = vx_base * math.cos(theta) - vy_base * math.sin(theta)
    vy_world = vx_base * math.sin(theta) + vy_base * math.cos(theta)
    return np.array([vx_world, vy_world])

def filter_colliding_actions(
    robot_q: np.ndarray,
    robot_orientation_theta: float, # New
    robot_actions_base: List[np.ndarray], # Actions are in base_link frame
    humans_q: List[np.ndarray],
    humans_v: List[np.ndarray], # Assumed in world frame
    time_step: float,
    robot_radius: float,
    human_radius: float
) -> List[np.ndarray]:
    """
    Filters the robot's base_link action space, removing actions that lead to collision.
    Collision checking is done in the world frame.
    """
    v_cf_base = [] # Collision-free actions in base_link frame
    for action_base in robot_actions_base:
        # Transform action from base_link to world frame for collision checking
        action_world = _transform_base_to_world_velocity(action_base, robot_orientation_theta)
        
        collision_predicted = False
        for hq, hv_world in zip(humans_q, humans_v): # hv_world is already world frame
            # geo.check_collision expects robot_v_or_action in the same frame as robot_q (world)
            if geo.check_collision(robot_q, action_world, hq, hv_world, time_step, robot_radius, human_radius):
                collision_predicted = True
                break
        if not collision_predicted:
            v_cf_base.append(action_base) # Keep the original base_link action
    return v_cf_base


def update_reactive_agents(
    robot_q: np.ndarray,
    robot_orientation_theta: float, # New
    current_robot_velocity_base: np.ndarray, # In base_link frame
    humans_q: List[np.ndarray],
    humans_v: List[np.ndarray], # Assumed in world frame
    fov_deg: float = DEFAULT_FOV_DEG
) -> Tuple[List[np.ndarray], List[np.ndarray], List[int]]:
    """
    Identifies human agents within the robot's field of view.
    FOV is based on robot's world-frame direction of motion.
    """
    reactive_q = []
    reactive_v = []
    reactive_indices = []

    # Transform current robot velocity to world frame to determine world-frame direction
    current_robot_velocity_world = _transform_base_to_world_velocity(current_robot_velocity_base, robot_orientation_theta)
    robot_dir_world, robot_speed_world = geo.normalize(current_robot_velocity_world)

    if robot_speed_world < geo.EPSILON: # Use world speed
        return reactive_q, reactive_v, reactive_indices

    fov_rad_half = np.deg2rad(fov_deg) / 2.0

    for i, (hq, hv_world) in enumerate(zip(humans_q, humans_v)):
        vec_rh_world = np.asarray(hq) - np.asarray(robot_q) # Vector from robot to human in world frame
        vec_rh_world_normalized, dist_rh = geo.normalize(vec_rh_world)

        if dist_rh < geo.EPSILON:
            continue

        dot_product = np.dot(robot_dir_world, vec_rh_world_normalized)
        dot_product = np.clip(dot_product, -1.0, 1.0)
        angle = np.arccos(dot_product)

        if abs(angle) <= fov_rad_half:
            reactive_q.append(hq)
            reactive_v.append(hv_world) # Store world-frame human velocity
            reactive_indices.append(i)

    return reactive_q, reactive_v, reactive_indices


def calculate_efficiency_score(
    action_base: np.ndarray, # Action in base_link frame
    robot_q: np.ndarray,
    robot_orientation_theta: float, # New
    goal_q: np.ndarray,
    time_step: float
) -> float:
    """
    Calculates efficiency score. Robot's next position is predicted in world frame.
    """
    action_world = _transform_base_to_world_velocity(action_base, robot_orientation_theta)
    robot_q_next_world = np.asarray(robot_q) + action_world * time_step
    dist_to_goal = np.linalg.norm(robot_q_next_world - np.asarray(goal_q))
    return -dist_to_goal


def calculate_social_momentum_score(
    robot_action_base: np.ndarray, # Robot action in base_link frame
    robot_q: np.ndarray,
    robot_orientation_theta: float, # New
    current_robot_velocity_base: np.ndarray, # Robot's current velocity in base_link
    reactive_humans_q: List[np.ndarray], # World frame
    reactive_humans_v_world: List[np.ndarray], # World frame
    time_step: float
) -> float:
    """
    Calculates Social Momentum. All velocities for angular momentum are in world frame.
    """
    total_sm_score = 0.0
    
    # Transform robot velocities to world frame for momentum calculation
    world_robot_action = _transform_base_to_world_velocity(robot_action_base, robot_orientation_theta)
    world_current_robot_velocity = _transform_base_to_world_velocity(current_robot_velocity_base, robot_orientation_theta)

    robot_q_next_world = np.asarray(robot_q) + world_robot_action * time_step

    weights = []
    projected_momenta_z = []

    for i, (hq, hv_world) in enumerate(zip(reactive_humans_q, reactive_humans_v_world)):
        hq_next_world = np.asarray(hq) + np.asarray(hv_world) * time_step # Human velocity is already world

        # Current momentum L_rhi (using world velocities)
        L_current_z = geo.calculate_angular_momentum_z(robot_q, world_current_robot_velocity, hq, hv_world)

        # Projected momentum L^_rhi(vr) (using world velocities)
        L_projected_z = geo.calculate_angular_momentum_z(robot_q_next_world, world_robot_action, hq_next_world, hv_world)

        if L_current_z * L_projected_z < -geo.EPSILON:
            return 0.0

        dist = np.linalg.norm(robot_q - hq)
        weight = 1.0 / (dist + geo.EPSILON)
        weights.append(weight)
        projected_momenta_z.append(L_projected_z)

    total_weight = sum(weights)
    if total_weight < geo.EPSILON:
        return 0.0

    normalized_weights = [w / total_weight for w in weights]
    for i in range(len(reactive_humans_q)):
         total_sm_score += normalized_weights[i] * abs(projected_momenta_z[i])
    return total_sm_score


def select_social_momentum_action(
    robot_q: np.ndarray, # World frame
    robot_orientation_theta: float, # Current robot yaw
    current_robot_velocity_base: np.ndarray, # In base_link frame
    robot_goal_q: np.ndarray, # World frame
    all_humans_q: List[np.ndarray], # World frame
    all_humans_v: List[np.ndarray], # Assumed world frame from publisher
    robot_action_space_base: List[np.ndarray], # Actions in base_link frame
    lambda_sm: float,
    time_step: float,
    robot_radius: float,
    human_radius: float,
    fov_deg: float = DEFAULT_FOV_DEG
) -> np.ndarray: # Returns selected action in base_link frame
    """
    Selects the best robot action (in base_link frame) based on Social Momentum.
    Internal calculations requiring world frame transformations use robot_orientation_theta.
    """
    v_cf_base = filter_colliding_actions(
        robot_q, robot_orientation_theta, robot_action_space_base,
        all_humans_q, all_humans_v, time_step, robot_radius, human_radius
    )

    if not v_cf_base:
        return np.array([0.0, 0.0]) # Stop if no safe move (returns base_link action)

    reactive_q, reactive_v_world, _ = update_reactive_agents(
        robot_q, robot_orientation_theta, current_robot_velocity_base,
        all_humans_q, all_humans_v, fov_deg
    )

    best_action_base = None # Store the best action in base_link frame
    best_score = -np.inf

    if reactive_q: # If there are humans in FOV
        for action_base in v_cf_base:
            efficiency_score = calculate_efficiency_score(
                action_base, robot_q, robot_orientation_theta, robot_goal_q, time_step
            )
            sm_score = calculate_social_momentum_score(
                action_base, robot_q, robot_orientation_theta, current_robot_velocity_base,
                reactive_q, reactive_v_world, time_step
            )
            total_score = efficiency_score + lambda_sm * sm_score
            if total_score > best_score:
                best_score = total_score
                best_action_base = action_base
    else: # No humans in FOV, optimize for efficiency only
        for action_base in v_cf_base:
            efficiency_score = calculate_efficiency_score(
                action_base, robot_q, robot_orientation_theta, robot_goal_q, time_step
            )
            if efficiency_score > best_score:
                best_score = efficiency_score
                best_action_base = action_base

    if best_action_base is None and v_cf_base:
        best_action_base = v_cf_base[0] # Default to first safe base_link action
    elif best_action_base is None:
         best_action_base = np.array([0.0, 0.0]) # Fallback base_link stop action

    return np.asarray(best_action_base) # Return the chosen action in base_link frame