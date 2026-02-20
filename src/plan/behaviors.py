"""
    Charlie Robot - Behavior Tree Module
    Behavior Tree nodes and construction for robot decision making.

    This module contains:
    - Helper functions for navigation calculations
    - All Behavior Tree node classes
    - build_tree() function to construct the complete tree
"""

import math
import random
import time

import py_trees
from py_trees import decorators
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.common import Status, ParallelPolicy


####################################################################################
# KNOWN TARGETS CONFIGURATION
# Define the map locations and their coordinates
# Robot does NOT know which target is the valve - must check each one
# theta = robot orientation at arrival (radians, 0 = facing right, π/2 = facing up)
####################################################################################
KNOWN_TARGETS = {
    # Color-based targets - robot will visit these and check for valve
    'green': {'x': -0.5, 'y': 0.0, 'theta': 0.0}, #la x è 
    #'blue': {'x': 1.0, 'y': 1.0, 'theta': 0.0},
    #'red': {'x': -6.5, 'y': -3.0, 'theta': 3.14},  # This is actually the valve, but robot doesn't know
}

# HOME/SPAWN POSITION - Will be saved automatically when robot starts
# This is just a fallback default if robot_position is not available at startup
HOME_POSITION_DEFAULT = {
    'x': 0.0,
    'y': 0.0,
    'theta': 0.0
}
####################################################################################


#HELPER FUNCTIONS
def calculate_best_direction(distance_left, distance_center, distance_right, threshold=0.12):
    """
    Calculate the optimal movement direction based on 3 distance sensors.
    
    IR sensor values (from sense_node 4-band conversion):
        0.20m = far (no concern)
        0.12m = medium (valid detection)
        0.08m = medium-close (wall nearby)
        0.05m = close (danger)
    
    Logic:
    1. Safety: If ALL sensors <= 0.05m -> AVOID_OBSTACLE (Spin)
    2. Frontal Obstacle: If Center <= threshold -> Turn to side with MORE space.
    3. Center Clear:
       - Wall Following: If side <= 0.08m -> Turn AWAY from wall.
       - Else -> MOVE_FORWARD.
    """
    # 1. Safety Panic Check (Too close to everything)
    SAFETY_LIMIT = 0.05  # 5cm - danger zone from IR sensors
    if distance_left <= SAFETY_LIMIT and distance_center <= SAFETY_LIMIT and distance_right <= SAFETY_LIMIT:
        return 'AVOID_OBSTACLE'
    
    # 2. Frontal Obstacle Check (Primary Trigger)
    if distance_center <= threshold:
        # Center is blocked - we MUST turn.
        # Compare Left vs Right to find the best escape route.
        if distance_left > distance_right:
            return 'TURN_LEFT'  # Left has more space
        else:
            return 'TURN_RIGHT' # Right has more space
            
    # 3. Center is Clear - Wall Following / Side Safety
    WALL_THRESHOLD = 0.08  # 8cm - medium-close band = wall detected
    
    if distance_left <= WALL_THRESHOLD:
        return 'TURN_RIGHT' # Too close to left wall -> Align right
        
    if distance_right <= WALL_THRESHOLD:
        return 'TURN_LEFT'  # Too close to right wall -> Align left
        
    # Safe to move forward (follow wall)
    return 'MOVE_FORWARD'


def calculate_closest_target(robot_pos, visited_targets):
    """
        Find the closest unvisited target from KNOWN_TARGETS.
        
        Args:
            robot_pos: Dict with robot position {'x': float, 'y': float, 'theta': float}
            visited_targets: List of already visited target names
        
        Returns:
            Dict with target info {'name', 'x', 'y', 'theta', 'distance'} or None if all visited
    """
    robot_x = robot_pos.get('x', 0.0)
    robot_y = robot_pos.get('y', 0.0)
    
    closest = None
    min_distance = float('inf')
    
    #Only consider unique positions
    seen_positions = set()
    
    for target_name, target_data in KNOWN_TARGETS.items():
        #Skip visited targets
        if target_name in visited_targets:
            continue
        
        #Skip duplicate positions 
        pos_key = (target_data['x'], target_data['y'])
        if pos_key in seen_positions:
            continue
        seen_positions.add(pos_key)
        
        #Calculate Euclidean distance
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
            target: Dict {'x', 'y', 'theta'} - target position and orientation
        
        Returns:
            str: 'MOVE_FORWARD', 'TURN_LEFT', or 'TURN_RIGHT'
    """
    robot_x = robot_pos.get('x', 0.0)
    robot_y = robot_pos.get('y', 0.0)
    robot_theta = robot_pos.get('theta', 0.0)
    
    target_x = target.get('x', 0.0)
    target_y = target.get('y', 0.0)
    
    #Calculate angle to target
    dx = target_x - robot_x
    dy = target_y - robot_y
    
    target_angle = math.atan2(dy, dx)
    
    #Calculate angle difference (normalize to -π to π)
    angle_diff = target_angle - robot_theta
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    
    #Decide action based on angle difference
    angle_threshold = 0.1  # ~5 degrees
    
    if abs(angle_diff) < angle_threshold:
        return 'MOVE_FORWARD'
    elif angle_diff > 0:
        return 'MOVE_FRONT_LEFT'
    else:
        return 'MOVE_FRONT_RIGHT'


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


#BEHAVIOR TREE NODES
#Battery Management
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
        Charging station is located at spawn point.
        Uses 3-sensor direction calculation for navigation.
    """
    def __init__(self):
        super().__init__(name="GoCharge")
        self.bb = self.attach_blackboard_client(name=self.name)
        
        #Write access for battery, action, and goal
        self.bb.register_key("battery", access=py_trees.common.Access.WRITE)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("goal_pose", access=py_trees.common.Access.WRITE)
        
        #Read access for distance sensors and home position
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_center", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
        self.bb.register_key("home_position", access=py_trees.common.Access.READ)
    
    def update(self):
        battery = self.bb.get("battery")
        
        #Read distance sensors
        distance_left = self.bb.get("distance_left") or 999.0
        distance_center = self.bb.get("distance_center") or 999.0
        distance_right = self.bb.get("distance_right") or 999.0
        
        #Set goal to charging station (spawn platform - saved at startup)
        home_pos = self.bb.get("home_position") or HOME_POSITION_DEFAULT
        goal_pose = {
            'x': home_pos['x'],
            'y': home_pos['y'],
            'theta': home_pos['theta']
        }
        self.bb.set("goal_pose", goal_pose)
        
        print(f"[PLAN] BATTERY LOW - returning to charge station ({home_pos['x']:.2f}, {home_pos['y']:.2f})")
        
        #Calculate optimal direction based on 3 sensors
        action = calculate_best_direction(distance_left, distance_center, distance_right)
        self.bb.set("plan_action", action)
        
        #Simulate charging (random charge amount)
        charge_amount = random.randint(30, 100 - int(battery))
        new_battery = min(100, battery + charge_amount)
        self.bb.set("battery", new_battery)
        
        #SUCCESS when fully charged (>= 80%), otherwise RUNNING
        return Status.SUCCESS if new_battery >= 80 else Status.RUNNING


#Target Navigation
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
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("odom_correction", access=py_trees.common.Access.READ)
        self.bb.register_key("detected_color", access=py_trees.common.Access.WRITE)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self._last_logged_target = None  #Evita spam di log

    def update(self):
        #1. Check if we already have a valid pending target (Persistence)
        current = self.bb.get("current_target")
        visited = self.bb.get("visited_targets") or []
        
        if current and current['name'] not in visited:
             #Keep current target until visited
             return Status.SUCCESS
             
        robot_pos_raw = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
        #Apply odometry correction if available
        odom_correction = self.bb.get("odom_correction") or {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0}
        robot_pos = {
            'x': robot_pos_raw.get('x', 0.0) + odom_correction.get('dx', 0.0),
            'y': robot_pos_raw.get('y', 0.0) + odom_correction.get('dy', 0.0),
            'theta': robot_pos_raw.get('theta', 0.0) + odom_correction.get('dtheta', 0.0)
        }
        
        #2. Select NEW closest unvisited target
        closest_name = None
        closest_dist = float('inf')
        closest_data = None
        
        for name, data in KNOWN_TARGETS.items():
            if name not in visited:
                dist = (data['x'] - robot_pos['x'])**2 + (data['y'] - robot_pos['y'])**2
                if dist < closest_dist:
                    closest_dist = dist
                    closest_name = name
                    closest_data = data
        
        if closest_name:
            self.bb.set("current_target", {
                'name': closest_name,
                'x': closest_data['x'],
                'y': closest_data['y'],
                'theta': closest_data['theta']
            })
            # Log only when target changes
            if closest_name != self._last_logged_target:
                print(f"[PLAN] NEW TARGET: {closest_name.upper()} @ ({closest_data['x']:.2f}, {closest_data['y']:.2f}) | Dist: {math.sqrt(closest_dist):.2f}m")
                self._last_logged_target = closest_name
            return Status.SUCCESS
        
        # All targets visited
        #print(f"[PLAN] ALL TARGETS VISITED: {visited}")
        return Status.FAILURE


class AtTarget(py_trees.behaviour.Behaviour):
    """
        Check if robot has arrived at the target.
        
        Complete Flow:
        1. Color Detection (< 1m): Center the color in camera
           - If left/right: turn until centered
           - Once centered: go straight
        2. Proximity Detection (front sensor < 0.20m):
           - Apply x/y odometry correction
           - Mark target as visited, proceed to next target
    """
    COLOR_DETECTION_DISTANCE = 1.0  # Distance threshold for color detection (meters)
    PROXIMITY_THRESHOLD = 0.08  # Distance to consider target reached (between 0.05 danger and 0.12 medium)
    
    def __init__(self):
        super().__init__(name="AtTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("current_target", access=py_trees.common.Access.WRITE)
        self.bb.register_key("detected_color", access=py_trees.common.Access.READ)
        self.bb.register_key("visited_targets", access=py_trees.common.Access.WRITE)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("found", access=py_trees.common.Access.WRITE)
        self.bb.register_key("odom_correction", access=py_trees.common.Access.WRITE)
        self.bb.register_key("detection_zone", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_center", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)

        # Internal state
        self._color_detected_logged = False 
        self._centering_complete = False

    def update(self):
        target = self.bb.get("current_target")
        if not target:
            return Status.FAILURE
        
        # Get robot position with odometry correction
        robot_pos_raw = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        odom_correction = self.bb.get("odom_correction") or {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0}
        
        robot_pos = {
            'x': robot_pos_raw.get('x', 0.0) + odom_correction.get('dx', 0.0),
            'y': robot_pos_raw.get('y', 0.0) + odom_correction.get('dy', 0.0),
            'theta': robot_pos_raw.get('theta', 0.0) + odom_correction.get('dtheta', 0.0)
        }
        
        robot_x = robot_pos.get('x', 0.0)
        robot_y = robot_pos.get('y', 0.0)
        
        # Calculate distance to target
        dx = target['x'] - robot_x
        dy = target['y'] - robot_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        # Get all 3 proximity sensor readings
        d_left = self.bb.get("distance_left") or 999.0
        d_center = self.bb.get("distance_center") or 999.0
        d_right = self.bb.get("distance_right") or 999.0
        
        # Check if ANY sensor detects target close enough
        min_distance = min(d_left, d_center, d_right)
        target_close = min_distance < self.PROXIMITY_THRESHOLD
        
        detected_color = self.bb.get("detected_color")
        target_name = target.get("name")
        detection_zone = self.bb.get("detection_zone")
        
        # Check if detected color matches the current target
        color_matches_target = (detected_color and detected_color.lower() == target_name.lower())
        
        # ============================================================================
        # TARGET REACHED: Apply x/y correction and complete
        # ============================================================================
        if target_close and color_matches_target:
            print(f"[PLAN] TARGET REACHED: {target_name.upper()} (sensor: {min_distance:.2f}m) | Color: {detected_color}")
            
            # Calculate and apply odometry correction (x/y only, no theta)
            target_coords = KNOWN_TARGETS.get(target_name, {})
            
            new_correction = {
                'dx': target_coords.get('x', 0.0) - robot_pos_raw.get('x', 0.0),
                'dy': target_coords.get('y', 0.0) - robot_pos_raw.get('y', 0.0),
                'dtheta': 0.0  # No theta correction
            }
            
            self.bb.set("odom_correction", new_correction)
            
            print(f"[ODOM] CORRECTION APPLIED at {target_name.upper()}:")
            print(f"[ODOM]   Raw pos: ({robot_pos_raw.get('x', 0):.2f}, {robot_pos_raw.get('y', 0):.2f})")
            print(f"[ODOM]   Known:   ({target_coords.get('x', 0):.2f}, {target_coords.get('y', 0):.2f})")
            print(f"[ODOM]   Offset:  (dx={new_correction['dx']:.2f}, dy={new_correction['dy']:.2f})")
            
            # Check if valve
            if detected_color == "red":
                self.bb.set("found", "valve")
                print(f"[PLAN] VALVE FOUND!")
            
            # Mark target as visited
            visited = self.bb.get("visited_targets") or []
            if target_name not in visited:
                visited.append(target_name)
                self.bb.set("visited_targets", visited)

            self.bb.set("plan_action", "STOP")
            
            # Reset state for next target
            self.bb.set("current_target", None)
            self._color_detected_logged = False
            self._centering_complete = False
            
            return Status.SUCCESS
        
        # ============================================================================
        # COLOR DETECTION & CENTERING (< 1 meter, correct color only)
        # ============================================================================
        if color_matches_target and distance_to_target <= self.COLOR_DETECTION_DISTANCE:
            # Log once when color first detected
            if not self._color_detected_logged:
                print(f"[PLAN] COLOR DETECTED: {detected_color.upper()} at {target_name.upper()} (dist: {distance_to_target:.2f}m)")
                self._color_detected_logged = True 
                self._centering_complete = False
            
            # Center color in camera frame
            if detection_zone and detection_zone != 'center':
                if detection_zone == 'left':
                    self.bb.set("plan_action", "MOVE_FRONT_LEFT")
                elif detection_zone == 'right':
                    self.bb.set("plan_action", "MOVE_FRONT_RIGHT")
                self._centering_complete = False
                return Status.RUNNING
            else:
                # Color is centered - advance straight
                if not self._centering_complete:
                    print(f"[PLAN] COLOR CENTERED - advancing to {target_name.upper()}...")
                    self._centering_complete = True
                
                self.bb.set("plan_action", "MOVE_FORWARD")
                return Status.RUNNING
        
        # Reset centering when color is lost or too far
        if not color_matches_target or distance_to_target > self.COLOR_DETECTION_DISTANCE:
            self._color_detected_logged = False
            self._centering_complete = False
        
        # Not at target yet
        return Status.FAILURE


class InitialRetreat(py_trees.behaviour.Behaviour):
    """
    Startup sequence with 3 phases:
    1. RETREAT: Move backward 0.5m to get away from base station
    2. ROTATE: Turn left 180° to face the room
    3. RESET_ORIGIN: Set current position as (0,0,0) origin
    
    Home/base station position is saved by plan_node BEFORE this runs.
    """
    RETREAT_DISTANCE = 0.5  # meters to retreat
    ROTATE_ANGLE = math.pi  # 180 degrees
    
    def __init__(self):
        super().__init__(name="InitialRetreat")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("startup_complete", access=py_trees.common.Access.WRITE)
        self.bb.register_key("startup_complete", access=py_trees.common.Access.READ)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("origin_offset", access=py_trees.common.Access.WRITE)
        
        self._phase = "RETREAT"  # RETREAT -> ROTATE -> RESET_ORIGIN
        self._start_position = None
        self._rotate_start_theta = None
        self._total_rotated = 0.0
        self._last_theta = None
    
    def update(self):
        if self.bb.get("startup_complete"):
            return Status.SUCCESS
        
        robot_pos = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
        # ===== PHASE 1: RETREAT =====
        if self._phase == "RETREAT":
            if self._start_position is None:
                self._start_position = robot_pos.copy()
                print(f"[STARTUP] Phase 1: RETREAT ({self.RETREAT_DISTANCE}m backward)...")
            
            dx = robot_pos['x'] - self._start_position['x']
            dy = robot_pos['y'] - self._start_position['y']
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.RETREAT_DISTANCE:
                self.bb.set("plan_action", "MOVE_BACKWARD")
                return Status.RUNNING
            else:
                print(f"[STARTUP] Phase 1 DONE: retreated {distance:.2f}m")
                self._phase = "ROTATE"
                self._rotate_start_theta = robot_pos['theta']
                self._last_theta = robot_pos['theta']
                self._total_rotated = 0.0
                self.bb.set("plan_action", "STOP")
                return Status.RUNNING
        
        # ===== PHASE 2: ROTATE 180° =====
        if self._phase == "ROTATE":
            if self._rotate_start_theta is None:
                self._rotate_start_theta = robot_pos['theta']
                self._last_theta = robot_pos['theta']
                print(f"[STARTUP] Phase 2: ROTATE 180°...")
            
            # Track incremental rotation (handles wrap-around)
            current_theta = robot_pos['theta']
            delta = current_theta - self._last_theta
            
            # Normalize delta to [-pi, pi] for wrap-around
            while delta > math.pi:
                delta -= 2 * math.pi
            while delta < -math.pi:
                delta += 2 * math.pi
            
            self._total_rotated += abs(delta)
            self._last_theta = current_theta
            
            if self._total_rotated < self.ROTATE_ANGLE:
                self.bb.set("plan_action", "TURN_LEFT")
                return Status.RUNNING
            else:
                print(f"[STARTUP] Phase 2 DONE: rotated {math.degrees(self._total_rotated):.0f}°")
                self._phase = "RESET_ORIGIN"
                self.bb.set("plan_action", "STOP")
                return Status.RUNNING
        
        # ===== PHASE 3: RESET ORIGIN =====
        if self._phase == "RESET_ORIGIN":
            # Current robot_position already has the old offset applied.
            # We need to set origin_offset = current raw position
            # so that corrected position = raw - offset = (0, 0, 0)
            # Since current offset is (0,0,0) at startup, robot_position IS the raw position
            origin = {
                'x': robot_pos['x'],
                'y': robot_pos['y'],
                'theta': robot_pos['theta']
            }
            self.bb.set("origin_offset", origin)
            self.bb.set("startup_complete", True)
            self.bb.set("plan_action", "STOP")
            print(f"[STARTUP] Phase 3 DONE: ORIGIN RESET")
            print(f"[STARTUP] Raw position was ({origin['x']:.2f}, {origin['y']:.2f}, {math.degrees(origin['theta']):.0f}°)")
            print(f"[STARTUP] New origin = (0.00, 0.00, 0°) - Navigation started!")
            return Status.SUCCESS
        
        return Status.RUNNING




class MoveToTarget(py_trees.behaviour.Behaviour):
    """
        Navigate towards the target with obstacle avoidance.
        Uses Hysteresis-based navigation to avoid oscillation.
        
        Logic Flow:
        1. Obstacle Detection (Ultrasonic)
        - If any sensor blocked: Turn away from obstacle.
        - Use calculate_best_direction() for smart choice.
        2. Recovery (Post-Avoidance)
        - If was avoiding and now clear: Move Forward blindly (1-2s) to pass obstacle.
        3. Visual Search (when near target position)
        - If close to target position but color not seen: Rotate to scan.
        4. Navigation (Target)
        - If clear: Calculate angle error.
        - Only change direction if error is large (> threshold).
        - Maintain current direction otherwise (Hysteresis).
    """
    SEARCH_DISTANCE =  0.12 # Start visual search when within this distance (meters)
    
    def __init__(self):
        super().__init__(name="MoveToTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        
        #Write access
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("goal_pose", access=py_trees.common.Access.WRITE)
        
        #Read access
        self.bb.register_key("current_target", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_center", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("odom_correction", access=py_trees.common.Access.READ)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
        self.bb.register_key("detection_zone", access=py_trees.common.Access.READ)
        self.bb.register_key("detection_distance", access=py_trees.common.Access.READ)
        self.bb.register_key("detection_confidence", access=py_trees.common.Access.READ)
        self.bb.register_key("detected_color", access=py_trees.common.Access.READ)
        
        #Internal State
        self.avoiding = False
        self.recovery_steps = 0
        self.last_action = "STOP"
        self._debug_tick = 0  #Contatore per logging periodico
        self._search_ticks = 0  # Count ticks while searching
        self._search_logged = False

    def update(self):
        #Get Target (Persistent)
        #Fallisce 
        target = self.bb.get("current_target")
        if not target:
            self.bb.set("plan_action", "STOP")
            return Status.FAILURE  #Should trigger CalculateTarget

        #Update Blackboard Goal (for logging/debug)
        self.bb.set("goal_pose", target)

        #Read Sensors & Pos
        d_left = self.bb.get("distance_left") or 999.0
        d_center = self.bb.get("distance_center") or 999.0
        d_right = self.bb.get("distance_right") or 999.0
        robot_pos_raw = self.bb.get("robot_position") or {'x': 0, 'y': 0, 'theta': 0}
        
        #Apply odometry correction if available
        odom_correction = self.bb.get("odom_correction") or {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0}
        robot_pos = {
            'x': robot_pos_raw.get('x', 0.0) + odom_correction.get('dx', 0.0),
            'y': robot_pos_raw.get('y', 0.0) + odom_correction.get('dy', 0.0),
            'theta': robot_pos_raw.get('theta', 0.0) + odom_correction.get('dtheta', 0.0)
        }
        
        # Calculate distance to target (odometry-based)
        robot_x = robot_pos.get('x', 0.0)
        robot_y = robot_pos.get('y', 0.0)
        robot_theta = robot_pos.get('theta', 0.0)
        dx = target['x'] - robot_x
        dy = target['y'] - robot_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        # Get detected color for visual guidance
        detected_color = self.bb.get("detected_color")
        target_name = target.get("name")
        color_matches_target = (detected_color and detected_color.lower() == target_name.lower())
        detection_zone = self.bb.get("detection_zone")
        
        #Thresholds (adapted for IR sensor 4-band values: 0.20, 0.12, 0.08, 0.05)
        OBSTACLE_DIST = 0.12  # medium range from IR sensors
        
        #CHECK FOR CHARGE_COLOR MODE (Ignore obstacles, go straight to colored wall)
        current_action = self.bb.get("plan_action")
        if current_action == "CHARGE_COLOR":
            self.bb.set("plan_action", "MOVE_FORWARD")
            if self._debug_tick % 30 == 0:
                print(f"[AVOID] CHARGE_COLOR active - ignoring obstacles, straight to {target['name'].upper()}")
            self._debug_tick += 1
            return Status.RUNNING
        
        #LOGIC START
        
        # OBSTACLE AVOIDANCE - Check if human detected for larger safety margin
        found = self.bb.get("found")
        human_detected = (found == "person")
        
        # Thresholds for 4-band IR (0.20 far, 0.12 medium, 0.08 wall, 0.05 danger)
        if human_detected:
            FRONT_OBSTACLE_DIST = 0.20  # react at far range for humans
            SIDE_OBSTACLE_DIST = 0.12   # medium range for humans
        else:
            FRONT_OBSTACLE_DIST = 0.12  # medium range = obstacle detected
            SIDE_OBSTACLE_DIST = 0.08   # wall detected (medium-close band)
        
        is_blocked = (d_center < FRONT_OBSTACLE_DIST or 
                      d_left < SIDE_OBSTACLE_DIST or 
                      d_right < SIDE_OBSTACLE_DIST)
        
        if is_blocked:
            was_avoiding = self.avoiding
            self.avoiding = True
            self.recovery_steps = 0
            
            action = calculate_best_direction(d_left, d_center, d_right, FRONT_OBSTACLE_DIST)
            
            # Log on first detection or periodically
            if not was_avoiding or self._debug_tick % 10 == 0:
                blocked_sensors = []
                if d_left < SIDE_OBSTACLE_DIST: blocked_sensors.append(f"L={d_left:.2f}<{SIDE_OBSTACLE_DIST}")
                if d_center < FRONT_OBSTACLE_DIST: blocked_sensors.append(f"C={d_center:.2f}<{FRONT_OBSTACLE_DIST}")
                if d_right < SIDE_OBSTACLE_DIST: blocked_sensors.append(f"R={d_right:.2f}<{SIDE_OBSTACLE_DIST}")
                prefix = "[AVOID HUMAN]" if human_detected else "[AVOID]"
                print(f"{prefix} ULTRASONIC: {' '.join(blocked_sensors)} -> {action}")
            
            self.last_action = action
            self.bb.set("plan_action", action)
            self._debug_tick += 1
            return Status.RUNNING

        #C. RECOVERY (Post-Avoidance) - Check if path to TARGET is clear, not just if obstacle is gone
        if self.avoiding:
            # Calculate angle to target
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - robot_theta
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Determine which sensor is relevant for the target direction
            # If target is ahead (|angle| < 45°), check front sensor
            # If target is left (angle > 45°), check left sensor  
            # If target is right (angle < -45°), check right sensor
            CLEAR_THRESHOLD = FRONT_OBSTACLE_DIST * 1.3  # Need a bit more clearance
            
            if abs(angle_diff) < 0.8:  # ~45° - target is roughly ahead
                path_clear = d_center > CLEAR_THRESHOLD
            elif angle_diff > 0:  # target is to the left
                path_clear = d_left > CLEAR_THRESHOLD and d_center > FRONT_OBSTACLE_DIST
            else:  # target is to the right
                path_clear = d_right > CLEAR_THRESHOLD and d_center > FRONT_OBSTACLE_DIST
            
            if path_clear:
                # Path to target is clear - resume navigation immediately
                self.avoiding = False
                self.recovery_steps = 0
                print(f"[AVOID] PATH TO {target['name'].upper()} CLEAR - resuming navigation")
            else:
                # Path still blocked - continue avoidance
                action = calculate_best_direction(d_left, d_center, d_right, FRONT_OBSTACLE_DIST)
                self.bb.set("plan_action", action)
                return Status.RUNNING
        
        # ============================================================================
        # VISUAL SEARCH: When close to target but color not detected, rotate to scan
        # ============================================================================
        if distance_to_target < self.SEARCH_DISTANCE and not color_matches_target:
            # Near target position but don't see the correct color - scan for it
            if not self._search_logged:
                print(f"[PLAN] SEARCH MODE: Near {target_name.upper()} (odom: {distance_to_target:.2f}m) - scanning...")
                self._search_logged = True
            
            # If we see the color but it's not centered, turn toward it
            if color_matches_target and detection_zone:
                if detection_zone == 'left':
                    action = 'TURN_LEFT'
                elif detection_zone == 'right':
                    action = 'TURN_RIGHT'
                else:
                    action = 'MOVE_FORWARD'
                self._search_ticks = 0
            else:
                # Don't see target color - rotate to scan
                action = 'TURN_LEFT'
                self._search_ticks += 1
                
                # Log periodically during search
                if self._search_ticks % 20 == 0:
                    print(f"[PLAN] SCANNING for {target_name.upper()}... (ticks: {self._search_ticks})")
            
            self.bb.set("plan_action", action)
            self._debug_tick += 1
            return Status.RUNNING
        else:
            # Reset search state when target found or far away
            self._search_logged = False
            self._search_ticks = 0
        
        #D. NAVIGATION (Hysteresis)
        #Calculate angle error to target
        target_angle = math.atan2(dy, dx)
        
        angle_diff = target_angle - robot_theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        #Hysteresis thresholds
        HYSTERESIS_THRESHOLD = 0.4  #~23 degrees - switch action only if error is big
        ALIGNMENT_THRESHOLD = 0.2   #~11 degrees - considered aligned with target
        
        #Decision logic with hysteresis
        if abs(angle_diff) < ALIGNMENT_THRESHOLD:
            # Well aligned - go straight
            action = 'MOVE_FORWARD'
        elif abs(angle_diff) > HYSTERESIS_THRESHOLD:
            # Big error - need to correct
            if angle_diff > 0:
                action = 'MOVE_FRONT_LEFT'
            else:
                action = 'MOVE_FRONT_RIGHT'
        else:
            # Medium error - maintain previous action (hysteresis)
            if self.last_action in ['MOVE_FORWARD', 'MOVE_FRONT_LEFT', 'MOVE_FRONT_RIGHT']:
                action = self.last_action
            else:
                #Was doing something else (avoidance) - pick appropriate
                action = 'MOVE_FORWARD'
        
        # Log navigation status every 5 seconds (50 ticks at 10Hz)
        if self._debug_tick % 50 == 0:
            angle_deg = math.degrees(angle_diff)
            print(f"[PLAN] NAV -> {target['name'].upper()} | Dist: {distance_to_target:.2f}m | Ang: {angle_deg:+.0f}deg | {action}")
        
        self._debug_tick += 1
        self.last_action = action
        self.bb.set("plan_action", action)
        return Status.RUNNING


#Object Detection & Recognition
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
        
        #Priority 1: Red color = valve (mission objective)
        if detected_color and detected_color.lower() == "red":
            self.bb.set("found", "valve")
        #Priority 2: Explicit valve from detections
        elif detections.get("valve"):
            self.bb.set("found", "valve")
        #Priority 3: Person detected
        elif detections.get("person"):
            self.bb.set("found", "person")
        #Priority 4: Generic obstacle
        elif detections.get("obstacle"):
            self.bb.set("found", "obstacle")
        else:
            self.bb.set("found", None)
        
        return Status.SUCCESS


class RecognitionPerson(py_trees.behaviour.Behaviour):
    """
        Check if 'found' equals 'person'. Returns SUCCESS if person found.
    """
    def __init__(self):
        super().__init__(name="RecognitionPerson")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.bb.get("found") == "person" else Status.FAILURE


class RecognitionObstacle(py_trees.behaviour.Behaviour):
    """
        Check if 'found' equals 'obstacle'. Returns SUCCESS if obstacle found.
    """
    def __init__(self):
        super().__init__(name="RecognitionObstacle")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.bb.get("found") == "obstacle" else Status.FAILURE


class RecognitionValve(py_trees.behaviour.Behaviour):
    """
        Check if 'found' equals 'valve'. Returns SUCCESS if valve found.
    """
    def __init__(self):
        super().__init__(name="RecognitionValve")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
    
    def update(self):
        return Status.SUCCESS if self.bb.get("found") == "valve" else Status.FAILURE


#Actions & Signals
class SignalPerson(py_trees.behaviour.Behaviour):
    """
        Add 'PersonFound' signal to signals list.
    """
    def __init__(self):
        super().__init__(name="SignalPerson")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("signals", access=py_trees.common.Access.WRITE)
    
    def update(self):
        signals = self.bb.get("signals") or []
        signals.append("PersonFound")
        self.bb.set("signals", signals)
        print(f"[PLAN] PERSON DETECTED - SignalPerson: PersonFound added to signals")
        return Status.SUCCESS


class GoAroundP(py_trees.behaviour.Behaviour):
    """
        Smart avoidance for person - turn to the opposite side of where person is detected.
        Uses detection_zone from camera to decide direction.
        ALSO saves robot position as human_position on FIRST avoidance.
    """
    _human_position_saved = False  # Class variable to track if already saved
    
    def __init__(self):
        super().__init__(name="GoAroundP")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("detection_zone", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("odom_correction", access=py_trees.common.Access.READ)
        self.bb.register_key("human_position", access=py_trees.common.Access.WRITE)
    
    def update(self):
        # Save human position on FIRST avoidance only
        if not GoAroundP._human_position_saved:
            robot_pos_raw = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
            odom_correction = self.bb.get("odom_correction") or {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0}
            
            human_pos = {
                'x': robot_pos_raw.get('x', 0.0) + odom_correction.get('dx', 0.0),
                'y': robot_pos_raw.get('y', 0.0) + odom_correction.get('dy', 0.0),
                'theta': robot_pos_raw.get('theta', 0.0) + odom_correction.get('dtheta', 0.0)
            }
            self.bb.set("human_position", human_pos)
            GoAroundP._human_position_saved = True
            print(f"[PLAN] HUMAN POSITION SAVED @ ({human_pos['x']:.2f}, {human_pos['y']:.2f})")
        
        zone = self.bb.get("detection_zone")
        dist_left = self.bb.get("distance_left") or 999.0
        dist_right = self.bb.get("distance_right") or 999.0
    
        #Turn opposite to where person is detected
        if zone == 'left':
            #Person on left → turn right
            action = 'TURN_RIGHT'
        elif zone == 'right':
            #Person on right → turn left
            action = 'TURN_LEFT'
        else:
            #Person in center → turn to more open side
            if dist_left > dist_right:
                action = 'TURN_LEFT'
            else:
                action = 'TURN_RIGHT'
        
        self.bb.set("plan_action", action)
        return Status.SUCCESS


class GoAroundO(py_trees.behaviour.Behaviour):
    """
        Smart avoidance for obstacle - turn to the opposite side of where obstacle is detected.
        Uses detection_zone from camera to decide direction.
    """
    def __init__(self):
        super().__init__(name="GoAroundO")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("detection_zone", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
    
    def update(self):
        zone = self.bb.get("detection_zone")
        dist_left = self.bb.get("distance_left") or 999.0
        dist_right = self.bb.get("distance_right") or 999.0
        
        #Turn opposite to where obstacle is detected
        if zone == 'left':
            action = 'TURN_RIGHT'
        elif zone == 'right':
            action = 'TURN_LEFT'
        else:
            #Obstacle in center → turn to more open side
            if dist_left > dist_right:
                action = 'TURN_LEFT'
            else:
                action = 'TURN_RIGHT'
        
        self.bb.set("plan_action", action)
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
        print(f"")
        print(f"[PLAN] =====================================")
        print(f"[PLAN] MISSION COMPLETE - VALVE ACTIVATED!")
        print(f"[PLAN] =====================================")
        print(f"")
        
        return Status.SUCCESS


class GoToHuman(py_trees.behaviour.Behaviour):
    """
        Navigate back to human position using odometry + visual homing.
        
        3-Phase approach:
        1. RETREAT: Back away from wall (3 seconds)
        2. ODOM_NAV: Use odometry to navigate toward human_position
        3. VISUAL_APPROACH: When near human, use person detection for precise approach
    """
    RETREAT_TIME = 3.0  # Seconds to retreat before navigating
    HUMAN_TOLERANCE = 1.0  # Switch to visual homing when within this distance
    VISUAL_COMPLETE_DISTANCE = 0.5  # Ultrasonic distance to consider arrived
    
    # Hysteresis thresholds (same as MoveToTarget)
    HYSTERESIS_THRESHOLD = 0.4  # ~23 degrees
    ALIGNMENT_THRESHOLD = 0.2   # ~11 degrees
    
    def __init__(self):
        super().__init__(name="GoToHuman")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("human_position", access=py_trees.common.Access.READ)
        self.bb.register_key("odom_correction", access=py_trees.common.Access.READ)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("goal_pose", access=py_trees.common.Access.WRITE)
        self.bb.register_key("found", access=py_trees.common.Access.READ)
        self.bb.register_key("detection_zone", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_center", access=py_trees.common.Access.READ)
        
        # Internal state
        self._last_log_time = None
        self._retreat_start = None
        self._retreat_done = False
        self._visual_homing_active = False
        self._visual_search_ticks = 0
        self._last_action = "MOVE_FORWARD"
    
    def update(self):
        # Get human position (saved by GoAroundP on first avoidance)
        human_pos = self.bb.get("human_position")
        if not human_pos:
            # No human position saved - cannot navigate
            print(f"[PLAN] GO TO HUMAN - ERROR: No human_position saved!")
            self.bb.set("plan_action", "STOP")
            return Status.FAILURE
        
        human_x = human_pos['x']
        human_y = human_pos['y']
        
        # Get raw robot position and apply odometry correction
        robot_pos_raw = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        odom_correction = self.bb.get("odom_correction") or {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0}
        
        current_x = robot_pos_raw.get('x', 0.0) + odom_correction.get('dx', 0.0)
        current_y = robot_pos_raw.get('y', 0.0) + odom_correction.get('dy', 0.0)
        current_theta = robot_pos_raw.get('theta', 0.0) + odom_correction.get('dtheta', 0.0)
        
        dx = human_x - current_x
        dy = human_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # ================================================================
        # PHASE 1: RETREAT from wall
        # ================================================================
        if not self._retreat_done:
            if self._retreat_start is None:
                self._retreat_start = time.time()
                print(f"[PLAN] GO TO HUMAN - Phase 1: Retreating from wall...")
            
            elapsed = time.time() - self._retreat_start
            if elapsed < self.RETREAT_TIME:
                self.bb.set("plan_action", "MOVE_BACKWARD")
                return Status.RUNNING
            else:
                self._retreat_done = True
                print(f"[PLAN] GO TO HUMAN - Phase 2: Navigating to human @ ({human_x:.2f}, {human_y:.2f})")
        
        # ================================================================
        # PHASE 3: VISUAL APPROACH (when close enough or already active)
        # ================================================================
        if self._visual_homing_active or distance < self.HUMAN_TOLERANCE:
            if not self._visual_homing_active:
                self._visual_homing_active = True
                print(f"[PLAN] GO TO HUMAN - Phase 3: Visual approach to person...")
            
            found = self.bb.get("found")
            detection_zone = self.bb.get("detection_zone")
            d_center = self.bb.get("distance_center") or 999.0
            
            # SUCCESS: Close to person
            if d_center < self.VISUAL_COMPLETE_DISTANCE:
                self.bb.set("plan_action", "STOP")
                print(f"[PLAN] HUMAN REACHED via visual approach (ultrasonic: {d_center:.2f}m)")
                return Status.SUCCESS
            
            # PERSON DETECTED - center and approach
            if found == "person":
                self._visual_search_ticks = 0
                
                if detection_zone == 'left':
                    action = 'MOVE_FRONT_LEFT'
                elif detection_zone == 'right':
                    action = 'MOVE_FRONT_RIGHT'
                else:  # center
                    action = 'MOVE_FORWARD'
                
                if self._last_log_time is None or (time.time() - self._last_log_time) > 3.0:
                    self._last_log_time = time.time()
                    print(f"[PLAN] VISUAL APPROACH: Person detected ({detection_zone}) | US: {d_center:.2f}m | {action}")
                
                self.bb.set("plan_action", action)
                return Status.RUNNING
            
            # PERSON NOT DETECTED - scan for them
            self._visual_search_ticks += 1
            if self._visual_search_ticks % 30 == 1:
                print(f"[PLAN] VISUAL APPROACH: Scanning for person... (ticks: {self._visual_search_ticks})")
            
            self.bb.set("plan_action", "TURN_LEFT")
            return Status.RUNNING
        
        # ================================================================
        # PHASE 2: ODOMETRY NAVIGATION with hysteresis
        # Also check for person detection - switch to visual approach if found
        # ================================================================
        
        # Check for person while navigating (early visual lock)
        found = self.bb.get("found")
        if found == "person":
            self._visual_homing_active = True
            print(f"[PLAN] GO TO HUMAN - PERSON DETECTED during navigation! Switching to visual approach...")
            return Status.RUNNING
        
        if self._last_log_time is None or (time.time() - self._last_log_time) > 5.0:
            self._last_log_time = time.time()
            angle_deg = math.degrees(math.atan2(dy, dx) - current_theta)
            print(f"[PLAN] GO TO HUMAN -> ({human_x:.2f}, {human_y:.2f}) | Dist: {distance:.2f}m | Ang: {angle_deg:+.0f}deg")
        
        # Calculate angle to human
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - current_theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Hysteresis navigation
        if abs(angle_diff) < self.ALIGNMENT_THRESHOLD:
            action = 'MOVE_FORWARD'
        elif abs(angle_diff) > self.HYSTERESIS_THRESHOLD:
            action = 'MOVE_FRONT_LEFT' if angle_diff > 0 else 'MOVE_FRONT_RIGHT'
        else:
            action = self._last_action if self._last_action in ['MOVE_FORWARD', 'MOVE_FRONT_LEFT', 'MOVE_FRONT_RIGHT'] else 'MOVE_FORWARD'
        
        self._last_action = action
        self.bb.set("plan_action", action)
        return Status.RUNNING


#BEHAVIOR TREE CONSTRUCTION
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
            - GoToHuman
    """
    #Battery management: check battery, go charge if low
    battery = Selector("Battery", memory=False, children=[
        BatteryCheck(),
        GoCharge()
    ])
    
    #Person handling: recognize, signal, and avoid
    person_seq = Sequence("PersonHandler", memory=False, children=[
        RecognitionPerson(),
        SignalPerson(),
        GoAroundP()
    ])
    
    #Obstacle handling: recognize and avoid
    obstacle_seq = Sequence("ObstacleHandler", memory=False, children=[
        RecognitionObstacle(),
        GoAroundO()
    ])
    
    #Handle found objects (person has priority over obstacle)
    found_handler = Selector("FoundHandler", memory=False, children=[
        person_seq,
        obstacle_seq
    ])
    
    #Search sequence: search for objects, then handle if found
    search = Sequence("Search", memory=False, children=[
        SearchObj(),
        found_handler
    ])
    
    #Decorator to prevent search failure from blocking parallel
    search_dec = decorators.FailureIsSuccess(
        name="SearchDecorator",
        child=search
    )
    
    #Parallel movement with searching
    move_search = Parallel("MoveAndSearch", 
        policy=ParallelPolicy.SuccessOnAll(),
        children=[MoveToTarget(), search_dec]
    )
    
    #Go to target: check if at target, otherwise move with search
    goto = Selector("GoToTarget", memory=False, children=[
        AtTarget(),
        move_search
    ])
    
    #Target search loop: keep searching until valve is found
    #This inner loop repeats CalculateTarget → GoToTarget until RecognitionValve succeeds
    target_search = Sequence("TargetSearch", memory=False, children=[
        CalculateTarget(),
        goto,
        #SecondoRetreat(),
        RecognitionValve(),
    ])
    
    #Retry target search until valve is found (-1 = infinite retries)
    target_loop = decorators.Retry("TargetLoop", child=target_search, num_failures=-1)
    
    #Main mission sequence
    main = Sequence("Main", memory=True, children=[
        InitialRetreat(),
        battery,
        target_loop,  #Loop until valve found
        ActiveValve(),
        GoToHuman()
    ])
    
    #Retry until mission success
    root = decorators.Retry("LoopUntilSuccess", child=main, num_failures=-1)
    
    return root
