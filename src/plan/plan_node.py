"""
    Charlie Robot - Plan Module
    Behavior Tree-based decision making for ROS2
    
    This module implements the strategic decision layer for the Charlie Robot.
    It receives sensor data from Sense module, makes decisions using a Behavior Tree,
    and sends commands to the Act module for physical execution.

    Subscribes to:
        /sense/world_state   - JSON: {distance_left, distance_center, distance_right,
                                      detected_color, robot_position, detections, battery}

    Publishes to:
        /plan/action         - String: action command (MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, 
                                       AVOID_OBSTACLE, STOP, ACTIVATE_VALVE)
        /plan/goal_pose      - String: JSON goal {x, y, theta}
        /plan/signals        - String: JSON array of signals ["PersonFound", "ValveActivated"]
"""

import json
import random

# ROS2 imports with fallback for testing
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object

# Behavior Tree imports
import py_trees
from py_trees import decorators
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.common import Status, ParallelPolicy


##########################################################################
# KNOWN TARGETS CONFIGURATION
# Define the map locations and their coordinates
# Robot does NOT know which target is the valve - must check each one
# theta = robot orientation at arrival (radians, 0 = facing right, π/2 = facing up)
##########################################################################
KNOWN_TARGETS = {
    # Color-based targets - robot will visit these and check for valve
    'green': {'x': 2.0, 'y': 3.0, 'theta': 0.0},
    'blue': {'x': 5.0, 'y': 9.0, 'theta': 1.57},
    'red': {'x': 10.0, 'y': 15.0, 'theta': 0.0},  # This is actually the valve, but robot doesn't know
}
##########################################################################


def calculate_best_direction(distance_left, distance_center, distance_right, threshold=0.5):
    """
    Calculate the optimal movement direction based on 3 distance sensors.
    
    This function implements obstacle avoidance logic by analyzing readings
    from left, center, and right distance sensors.
    
    Args:
        distance_left: Distance reading from left sensor (meters)
        distance_center: Distance reading from center sensor (meters)  
        distance_right: Distance reading from right sensor (meters)
        threshold: Minimum safe distance to consider path clear (default 0.5m)
    
    Returns:
        str: One of 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT', or 'AVOID_OBSTACLE'
    """
    # If center is clear, move forward
    if distance_center > threshold:
        return 'MOVE_FORWARD'
    
    # Center blocked - choose direction with more space
    if distance_left > threshold and distance_left >= distance_right:
        return 'TURN_LEFT'
    elif distance_right > threshold:
        return 'TURN_RIGHT'
    else:
        # All sensors blocked - generic obstacle avoidance
        return 'AVOID_OBSTACLE'


def calculate_closest_target(robot_pos, visited_targets):
    """
    Find the closest unvisited target from KNOWN_TARGETS.
    
    Args:
        robot_pos: Dict with robot position {'x': float, 'y': float, 'theta': float}
        visited_targets: List of already visited target names
    
    Returns:
        Dict with target info {'name', 'x', 'y', 'theta', 'distance'} or None if all visited
    """
    import math
    
    robot_x = robot_pos.get('x', 0.0)
    robot_y = robot_pos.get('y', 0.0)
    
    closest = None
    min_distance = float('inf')
    
    # Only consider unique positions (avoid duplicates like 'green' and '1')
    seen_positions = set()
    
    for target_name, target_data in KNOWN_TARGETS.items():
        # Skip visited targets
        if target_name in visited_targets:
            continue
        
        # Skip duplicate positions
        pos_key = (target_data['x'], target_data['y'])
        if pos_key in seen_positions:
            continue
        seen_positions.add(pos_key)
        
        # Calculate Euclidean distance
        dx = target_data['x'] - robot_x
        dy = target_data['y'] - robot_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < min_distance:
            min_distance = distance
            closest = {
                'name': target_name,
                'x': target_data['x'],
                'y': target_data['y'],
                'theta': target_data['theta'],
                'distance': distance
            }
    
    return closest


def calculate_direction_to_target(robot_pos, target):
    """
    Calculate the steering direction to reach target.
    
    Args:
        robot_pos: Dict {'x', 'y', 'theta'} - current robot position and orientation
        target: Dict {'x', 'y'} - target position
    
    Returns:
        str: 'MOVE_FORWARD', 'TURN_LEFT', or 'TURN_RIGHT'
    """
    import math
    
    robot_x = robot_pos.get('x', 0.0)
    robot_y = robot_pos.get('y', 0.0)
    robot_theta = robot_pos.get('theta', 0.0)
    
    # Calculate angle to target
    dx = target['x'] - robot_x
    dy = target['y'] - robot_y
    target_angle = math.atan2(dy, dx)
    
    # Calculate angle difference (normalize to -π to π)
    angle_diff = target_angle - robot_theta
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    
    # Decide action based on angle difference
    angle_threshold = 0.15  # ~8.5 degrees tolerance
    
    if abs(angle_diff) < angle_threshold:
        return 'MOVE_FORWARD'
    elif angle_diff > 0:
        return 'TURN_LEFT'
    else:
        return 'TURN_RIGHT'


def check_wall_alignment(distance_left, distance_right, threshold=0.05):
    """
    Check if robot is aligned with wall (facing straight).
    
    When left and right sensors read approximately the same distance,
    the robot is perpendicular to the wall.
    
    Args:
        distance_left: Left sensor reading (meters)
        distance_right: Right sensor reading (meters)
        threshold: Maximum allowed difference (default 0.05m = 5cm)
    
    Returns:
        bool: True if aligned, False otherwise
    """
    if distance_left is None or distance_right is None:
        return False
    
    diff = abs(distance_left - distance_right)
    return diff <= threshold


# ============================================================================
# BEHAVIOR TREE NODES - Battery Management
# ============================================================================

class BatteryCheck(py_trees.behaviour.Behaviour):
    """
    Check if battery level is sufficient for operation.
    Returns SUCCESS if battery > 20%, FAILURE otherwise.
    """
    def __init__(self):
        super().__init__(name="BatteryCheck")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("battery", access=py_trees.common.Access.READ)
    
    def update(self):
        battery = self.bb.get("battery")
        return Status.SUCCESS if battery > 20 else Status.FAILURE


class GoCharge(py_trees.behaviour.Behaviour):
    """
    Navigate to charging station and recharge battery.
    Charging station is located at spawn point (0, 0).
    Uses 3-sensor direction calculation for navigation.
    """
    def __init__(self):
        super().__init__(name="GoCharge")
        self.bb = self.attach_blackboard_client(name=self.name)
        
        # Write access for battery, action, and goal
        self.bb.register_key("battery", access=py_trees.common.Access.WRITE)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("goal_pose", access=py_trees.common.Access.WRITE)
        
        # Read access for distance sensors
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_center", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
    
    def update(self):
        battery = self.bb.get("battery")
        
        # Read distance sensors
        distance_left = self.bb.get("distance_left") or 999.0
        distance_center = self.bb.get("distance_center") or 999.0
        distance_right = self.bb.get("distance_right") or 999.0
        
        # Set goal to charging station (spawn point)
        goal_pose = {
            'x': 0.0,
            'y': 0.0,
            'theta': 3.14159  # Facing backward (π radians = 180°)
        }
        self.bb.set("goal_pose", goal_pose)
        
        # Calculate optimal direction based on 3 sensors
        action = calculate_best_direction(distance_left, distance_center, distance_right)
        self.bb.set("plan_action", action)
        
        # Simulate charging (random charge amount)
        charge_amount = random.randint(30, 100 - int(battery))
        new_battery = min(100, battery + charge_amount)
        self.bb.set("battery", new_battery)
        
        # SUCCESS when fully charged (>= 80%), otherwise RUNNING
        return Status.SUCCESS if new_battery >= 80 else Status.RUNNING


# ============================================================================
# BEHAVIOR TREE NODES - Target Navigation
# ============================================================================

class CalculateTarget(py_trees.behaviour.Behaviour):
    """
    Select the next unvisited target from KNOWN_TARGETS.
    Iterates through targets sequentially and selects first unvisited.
    Returns FAILURE when all targets have been visited.
    """
    def __init__(self):
        super().__init__(name="CalculateTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("visited_targets", access=py_trees.common.Access.WRITE)
        self.bb.register_key("current_target", access=py_trees.common.Access.WRITE)

    def update(self):
        visited = self.bb.get("visited_targets") or []

        # Find first unvisited target
        for target_name, target_data in KNOWN_TARGETS.items():
            if target_name not in visited:
                # Select this target
                self.bb.set("current_target", {
                    'name': target_name,
                    'x': target_data['x'],
                    'y': target_data['y'],
                    'theta': target_data['theta']
                })
                return Status.SUCCESS

        # All targets already visited
        return Status.FAILURE


class AtTarget(py_trees.behaviour.Behaviour):
    """
    Check if robot has arrived at target and verify if it's the valve.
    
    Logic:
    1. Check if near target (sensors detecting wall)
    2. Align robot with wall (left == right distance)
    3. Check if detected_color == red (valve)
    4. If valve → SUCCESS, else mark visited and continue
    """
    def __init__(self):
        super().__init__(name="AtTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("current_target", access=py_trees.common.Access.READ)
        self.bb.register_key("detected_color", access=py_trees.common.Access.READ)
        self.bb.register_key("visited_targets", access=py_trees.common.Access.WRITE)
        self.bb.register_key("reset_odom", access=py_trees.common.Access.WRITE)
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_center", access=py_trees.common.Access.READ)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("found", access=py_trees.common.Access.WRITE)

    def update(self):
        target = self.bb.get("current_target")
        if not target:
            return Status.FAILURE
        
        # Read distance sensors
        dist_left = self.bb.get("distance_left")
        dist_right = self.bb.get("distance_right")
        dist_center = self.bb.get("distance_center")
        
        # Step 1: Check if we're near target (center sensor detects wall)
        near_wall_threshold = 0.8  # meters
        if dist_center is None or dist_center > near_wall_threshold:
            # Not near target yet
            return Status.FAILURE
        
        # Step 2: Check wall alignment (left == right)
        if not check_wall_alignment(dist_left, dist_right, threshold=0.08):
            # Not aligned - adjust robot orientation
            if dist_left is not None and dist_right is not None:
                if dist_left > dist_right:
                    self.bb.set("plan_action", "TURN_LEFT")
                else:
                    self.bb.set("plan_action", "TURN_RIGHT")
            return Status.RUNNING
        
        # Step 3: Aligned! Stop and check color
        self.bb.set("plan_action", "STOP")
        
        detected_color = self.bb.get("detected_color")
        target_name = target.get("name")
        
        # Reset odometry to known target position
        reset_pose = {
            'x': target['x'],
            'y': target['y'],
            'theta': target['theta']
        }
        self.bb.set("reset_odom", reset_pose)
        
        # Mark target as visited
        visited = self.bb.get("visited_targets") or []
        if target_name not in visited:
            visited.append(target_name)
            self.bb.set("visited_targets", visited)
        
        # Step 4: Check if this is the valve (red color)
        if detected_color == "red":
            # FOUND THE VALVE!
            self.bb.set("found", "valve")
            return Status.SUCCESS
        
        # Not the valve - continue to next target
        return Status.SUCCESS


class MoveToTarget(py_trees.behaviour.Behaviour):
    """
    Navigate towards closest target using odometry-based direction.
    
    Logic:
    - If sensors detect nearby obstacles → use obstacle avoidance
    - If no obstacles → calculate direction to closest target using odometry
    """
    def __init__(self):
        super().__init__(name="MoveToTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        
        # Write access for action, goal, current_target
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("goal_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key("current_target", access=py_trees.common.Access.WRITE)
        
        # Read access for sensors and position
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_center", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("visited_targets", access=py_trees.common.Access.READ)
    
    def update(self):
        # Read distance sensors (default to large value = no obstacle)
        distance_left = self.bb.get("distance_left") or 999.0
        distance_center = self.bb.get("distance_center") or 999.0
        distance_right = self.bb.get("distance_right") or 999.0
        
        # Get robot position and visited targets
        robot_pos = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        visited = self.bb.get("visited_targets") or []
        
        # Calculate closest unvisited target
        closest = calculate_closest_target(robot_pos, visited)
        
        if not closest:
            # All targets visited, go home
            self.bb.set("plan_action", "STOP")
            return Status.SUCCESS
        
        # Set current target
        self.bb.set("current_target", closest)
        
        # Set goal pose
        goal_pose = {
            'x': closest['x'],
            'y': closest['y'],
            'theta': closest['theta']
        }
        self.bb.set("goal_pose", goal_pose)
        
        # Check if obstacles nearby (need avoidance)
        obstacle_threshold = 0.6  # meters
        obstacles_nearby = (distance_center < obstacle_threshold or 
                           distance_left < obstacle_threshold or 
                           distance_right < obstacle_threshold)
        
        if obstacles_nearby:
            # Use sensor-based obstacle avoidance
            action = calculate_best_direction(distance_left, distance_center, distance_right)
        else:
            # No obstacles - calculate direction to target using odometry
            action = calculate_direction_to_target(robot_pos, closest)
        
        self.bb.set("plan_action", action)
        return Status.RUNNING


# ============================================================================
# BEHAVIOR TREE NODES - Object Detection & Recognition
# ============================================================================

class SearchObj(py_trees.behaviour.Behaviour):
    """
    Search for objects using camera detections and color detection.
    Priority order:
    1. Red color = valve (mission objective)
    2. Explicit valve detection
    3. Person detection
    4. Obstacle detection
    """
    def __init__(self):
        super().__init__(name="SearchObj")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("detections", access=py_trees.common.Access.READ)
        self.bb.register_key("found", access=py_trees.common.Access.WRITE)
        self.bb.register_key("detected_color", access=py_trees.common.Access.READ)
    
    def update(self):
        detections = self.bb.get("detections") or {}
        detected_color = self.bb.get("detected_color")
        
        # Priority 1: Red color = valve (mission objective)
        if detected_color and detected_color.lower() == "red":
            self.bb.set("found", "valve")
        # Priority 2: Explicit valve from detections
        elif detections.get("valve"):
            self.bb.set("found", "valve")
        # Priority 3: Person detected
        elif detections.get("person"):
            self.bb.set("found", "person")
        # Priority 4: Generic obstacle
        elif detections.get("obstacle"):
            self.bb.set("found", "obstacle")
        else:
            self.bb.set("found", None)
        
        return Status.SUCCESS


class RecognitionPerson(py_trees.behaviour.Behaviour):
    """Check if 'found' equals 'person'. Returns SUCCESS if person found."""
    def __init__(self):
        super().__init__(name="RecognitionPerson")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.bb.get("found") == "person" else Status.FAILURE


class RecognitionObstacle(py_trees.behaviour.Behaviour):
    """Check if 'found' equals 'obstacle'. Returns SUCCESS if obstacle found."""
    def __init__(self):
        super().__init__(name="RecognitionObstacle")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.bb.get("found") == "obstacle" else Status.FAILURE


class RecognitionValve(py_trees.behaviour.Behaviour):
    """Check if 'found' equals 'valve'. Returns SUCCESS if valve found."""
    def __init__(self):
        super().__init__(name="RecognitionValve")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.bb.get("found") == "valve" else Status.FAILURE


# ============================================================================
# BEHAVIOR TREE NODES - Actions & Signals
# ============================================================================

class SignalPerson(py_trees.behaviour.Behaviour):
    """Add 'PersonFound' signal to signals list."""
    def __init__(self):
        super().__init__(name="SignalPerson")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("signals", access=py_trees.common.Access.WRITE)
    
    def update(self):
        signals = self.bb.get("signals") or []
        signals.append("PersonFound")
        self.bb.set("signals", signals)
        return Status.SUCCESS


class GoAroundP(py_trees.behaviour.Behaviour):
    """Set action to avoid person obstacle."""
    def __init__(self):
        super().__init__(name="GoAroundP")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
    
    def update(self):
        self.bb.set("plan_action", "AVOID_OBSTACLE")
        return Status.SUCCESS


class GoAroundO(py_trees.behaviour.Behaviour):
    """Set action to avoid generic obstacle."""
    def __init__(self):
        super().__init__(name="GoAroundO")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
    
    def update(self):
        self.bb.set("plan_action", "AVOID_OBSTACLE")
        return Status.SUCCESS


class ActiveValve(py_trees.behaviour.Behaviour):
    """
    Activate valve - mission objective complete.
    Adds 'ValveActivated' signal and sets mission_complete flag.
    """
    def __init__(self):
        super().__init__(name="ActiveValve")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("signals", access=py_trees.common.Access.WRITE)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("mission_complete", access=py_trees.common.Access.WRITE)
    
    def update(self):
        # Add valve activation signal
        signals = self.bb.get("signals") or []
        signals.append("ValveActivated")
        self.bb.set("signals", signals)
        
        # Set action and mission complete flag
        self.bb.set("plan_action", "ACTIVATE_VALVE")
        self.bb.set("mission_complete", True)
        return Status.SUCCESS


class GoHome(py_trees.behaviour.Behaviour):
    """
    Navigate back to home position (spawn point 0, 0) using odometry.
    Returns SUCCESS when within 0.3m of home, RUNNING otherwise.
    """
    def __init__(self):
        super().__init__(name="GoHome")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("goal_pose", access=py_trees.common.Access.WRITE)
    
    def update(self):
        # Home position = spawn point (0, 0)
        home_x, home_y = 0.0, 0.0
        
        # Get current robot position from odometry
        robot_pos = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        current_x = robot_pos.get('x', 0.0)
        current_y = robot_pos.get('y', 0.0)
        
        # Calculate Euclidean distance to home
        distance = ((current_x - home_x)**2 + (current_y - home_y)**2)**0.5
        
        # If close to home (< 0.3m), stop
        if distance < 0.3:
            self.bb.set("plan_action", "STOP")
            return Status.SUCCESS
        
        # Otherwise, set goal to home and continue moving
        goal_pose = {
            'x': home_x,
            'y': home_y,
            'theta': 0.0
        }
        self.bb.set("goal_pose", goal_pose)
        self.bb.set("plan_action", "MOVE_TO_GOAL")
        return Status.RUNNING


# ============================================================================
# BEHAVIOR TREE CONSTRUCTION
# ============================================================================

def build_tree():
    """
    Build the complete Behavior Tree for Charlie Robot.
    
    Tree Structure:
    - LoopUntilSuccess (Retry decorator)
      - Main (Sequence)
        - Battery (Selector: BatteryCheck or GoCharge)
        - CalculateTarget
        - GoToTarget (Selector: AtTarget or MoveAndSearch)
          - MoveAndSearch (Parallel: MoveToTarget + SearchDecorator)
            - SearchDecorator (FailureIsSuccess)
              - Search (Sequence: SearchObj + FoundHandler)
                - FoundHandler (Selector: PersonHandler or ObstacleHandler)
        - RecognitionValve
        - ActiveValve
        - GoHome
    """
    
    # Battery management: check battery, go charge if low
    battery = Selector("Battery", memory=False, children=[
        BatteryCheck(),
        GoCharge()
    ])
    
    # Person handling: recognize, signal, and avoid
    person_seq = Sequence("PersonHandler", memory=False, children=[
        RecognitionPerson(),
        SignalPerson(),
        GoAroundP()
    ])
    
    # Obstacle handling: recognize and avoid
    obstacle_seq = Sequence("ObstacleHandler", memory=False, children=[
        RecognitionObstacle(),
        GoAroundO()
    ])
    
    # Handle found objects (person has priority over obstacle)
    found_handler = Selector("FoundHandler", memory=False, children=[
        person_seq,
        obstacle_seq
    ])
    
    # Search sequence: search for objects, then handle if found
    search = Sequence("Search", memory=False, children=[
        SearchObj(),
        found_handler
    ])
    
    # Decorator to prevent search failure from blocking parallel
    search_dec = decorators.FailureIsSuccess(
        name="SearchDecorator",
        child=search
    )
    
    # Parallel movement with searching
    move_search = Parallel("MoveAndSearch", 
        policy=ParallelPolicy.SuccessOnAll(),
        children=[MoveToTarget(), search_dec]
    )
    
    # Go to target: check if at target, otherwise move with search
    goto = Selector("GoToTarget", memory=False, children=[
        AtTarget(),
        move_search
    ])
    
    # Main mission sequence
    main = Sequence("Main", memory=True, children=[
        battery,
        CalculateTarget(),
        goto,
        RecognitionValve(),
        ActiveValve(),
        GoHome()
    ])
    
    # Retry until mission success
    root = decorators.Retry("LoopUntilSuccess", child=main, num_failures=-1)
    
    return root


# ============================================================================
# ROS2 NODE
# ============================================================================

class PlanNode(Node):
    """
    Plan ROS2 Node - Strategic decision layer for Charlie Robot.
    
    Receives sensor data from Sense module via /sense/world_state topic.
    Processes data through Behavior Tree.
    Publishes commands to Act module via /plan/action, /plan/goal_pose, /plan/signals.
    """
    
    def __init__(self):
        super().__init__('plan_node')
        self.get_logger().info('=== Plan Node Starting ===')
        
        # Build Behavior Tree
        self.tree = build_tree()
        self.bb = py_trees.blackboard.Client(name="PlanNode")
        
        # Register all blackboard keys with write access
        blackboard_keys = [
            'battery', 'obstacles', 'detections', 'targets',
            'current_target', 'visited_targets', 'found', 'signals',
            'plan_action', 'goal_pose', 'mission_complete',
            'detected_color', 'reset_odom',  # removed yolo_valve
            'distance_left', 'distance_center', 'distance_right',
            'robot_position'
        ]
        for key in blackboard_keys:
            self.bb.register_key(key, access=py_trees.common.Access.WRITE)
        
        # Initialize blackboard with default values
        self._init_blackboard()
        self.tree.setup_with_descendants()
        
        # ROS2 Subscriptions - Multiple topics for different data
        # /sense/world_state - general data (obstacles, battery)
        self.create_subscription(String, '/sense/world_state', self._world_cb, 10)
        # /sense/robot_position - odometry position {x, y, theta}
        self.create_subscription(String, '/sense/robot_position', self._position_cb, 10)
        # /sense/distances - 3 distance sensors {left, center, right}
        self.create_subscription(String, '/sense/distances', self._distances_cb, 10)
        # /sense/detections - YOLO detections {detected_color, yolo_valve, person, obstacle}
        self.create_subscription(String, '/sense/detections', self._detections_cb, 10)
        
        ##TODO: Update with actual formats from Giam and Sal implementation
        # ROS2 Publishers
        self.action_pub = self.create_publisher(String, '/plan/action', 10)
        self.goal_pub = self.create_publisher(String, '/plan/goal_pose', 10)
        self.signals_pub = self.create_publisher(String, '/plan/signals', 10)
        
        # Timer for BT tick (10 Hz)
        self.create_timer(0.1, self._tick)
        
        self.get_logger().info('Plan Node ready')
    
    def _init_blackboard(self):
        """Initialize blackboard with default values."""
        # Base data
        self.bb.set("battery", 100.0)
        self.bb.set("obstacles", [])
        self.bb.set("detections", {})
        
        # Target navigation
        self.bb.set("current_target", None)
        self.bb.set("visited_targets", [])
        
        # Object detection & signals
        self.bb.set("found", None)
        self.bb.set("signals", [])
        
        # Commands to Act module
        self.bb.set("plan_action", "IDLE")
        self.bb.set("goal_pose", None)
        self.bb.set("mission_complete", False)
        
        # Sensor data from Sense module
        self.bb.set("detected_color", None)
        self.bb.set("reset_odom", None)
        self.bb.set("distance_left", 999.0)
        self.bb.set("distance_center", 999.0)
        self.bb.set("distance_right", 999.0)
        self.bb.set("robot_position", {'x': 0.0, 'y': 0.0, 'theta': 0.0})
    
    def _world_cb(self, msg):
        """
        Callback for /sense/world_state topic.
        Parses JSON data from Sense module and updates blackboard.
        """
        try:
            data = json.loads(msg.data)
            
            # Base data
            self.bb.set("obstacles", data.get("obstacles", []))
            
            # Detections (person, valve, fire, obstacle)
            self.bb.set("detections", data.get("detections", {}))
            
            # Battery level
            self.bb.set("battery", data.get("battery", 100.0))
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse world_state: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in _world_cb: {e}")
    
    def _position_cb(self, msg):
        """
        Callback for /sense/robot_position topic.
        Receives robot odometry position: {x, y, theta}
        """
        try:
            data = json.loads(msg.data)
            self.bb.set("robot_position", data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse robot_position: {e}")
    
    def _distances_cb(self, msg):
        """
        Callback for /sense/distances topic.
        Receives 3 distance sensor readings: {left, center, right}
        """
        try:
            data = json.loads(msg.data)
            if 'left' in data:
                self.bb.set("distance_left", data['left'])
            if 'center' in data:
                self.bb.set("distance_center", data['center'])
            if 'right' in data:
                self.bb.set("distance_right", data['right'])
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse distances: {e}")
    
    def _detections_cb(self, msg):
        """
        Callback for /sense/detections topic.
        Receives YOLO detections: {detected_color, person}
        
        Detection logic:
        - detected_color: "red" = valve (mission objective), others = normal targets
        - person: True if YOLO detects a person
        - obstacle: inferred from distance sensors (no explicit detection needed)
        """
        try:
            data = json.loads(msg.data)
            
            # Color detected by camera (red = valve)
            if 'detected_color' in data:
                self.bb.set("detected_color", data['detected_color'])
                # If red, we found the valve!
                if data['detected_color'] == "red":
                    self.bb.set("found", "valve")
            
            # Person detection from YOLO
            detections = {}
            if data.get('person'):
                detections['person'] = True
                self.bb.set("found", "person")
            
            if detections:
                self.bb.set("detections", detections)
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse detections: {e}")
    
    def _tick(self):
        """
        Timer callback - tick Behavior Tree and publish outputs.
        Called at 10 Hz (every 0.1 seconds).
        """
        # Tick Behavior Tree once
        self.tree.tick_once()
        
        # Publish action command to Act module
        action = self.bb.get("plan_action") or "IDLE"
        msg = String()
        msg.data = action
        self.action_pub.publish(msg)
        
        # Publish goal pose if set
        goal = self.bb.get("goal_pose")
        if goal:
            goal_msg = String()
            goal_msg.data = json.dumps(goal)
            self.goal_pub.publish(goal_msg)
        
        # Publish signals if any, then clear
        signals = self.bb.get("signals") or []
        if signals:
            signals_msg = String()
            signals_msg.data = json.dumps(signals)
            self.signals_pub.publish(signals_msg)
            # Clear signals after publishing
            self.bb.set("signals", [])


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def main(args=None):
    """Main entry point for Plan Node."""
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 not available. Install rclpy to run this node.")
        return
    
    rclpy.init(args=args)
    node = PlanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
