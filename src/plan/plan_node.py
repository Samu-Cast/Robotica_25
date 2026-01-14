"""
Charlie Robot - Plan Module
Behavior Tree-based decision making for ROS2

This module implements the strategic decision layer for the Charlie Robot.
It receives sensor data from Sense module, makes decisions using a Behavior Tree,
and sends commands to the Act module for physical execution.

Subscribes to:
    /sense/proximity/front, front_left, front_right (Range) - ultrasonic distances
    /sense/odometry (Pose2D) - robot position {x, y, theta}
    /sense/detection (String JSON) - current detection event

Publishes to:
    /plan/command - String command for Act (Front, Left, Right, Stop)
    /plan/signals - JSON array of event signals ["PersonFound", "ValveActivated"]
"""

import json
import random
import math
import time

# ROS2 imports with fallback for testing
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Range
    from geometry_msgs.msg import Pose2D
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object

#Behavior Tree imports
import py_trees
from py_trees import decorators
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.common import Status, ParallelPolicy


##########################################################################
#KNOWN TARGETS CONFIGURATION
#Define the map locations and their coordinates
#Robot does NOT know which target is the valve - must check each one
#theta = robot orientation at arrival (radians, 0 = facing right, π/2 = facing up)
##########################################################################
KNOWN_TARGETS = {
    #Color-based targets - robot will visit these and check for valve
    'green': {'x': -3.35, 'y': -5.0, 'theta': -1.65},
    'blue': {'x': 0.35, 'y': -4.0, 'theta': 0.0},
    'red': {'x': -6.25, 'y': -1.35, 'theta': 3.0}, #This is actually the valve, but robot doesn't know
}

#HOME/SPAWN POSITION - Will be saved automatically when robot starts
#This is just a fallback default if robot_position is not available at startup
HOME_POSITION_DEFAULT = {
    'x': 0.0,
    'y': 0.0,
    'theta': 0.0
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
    #If center is clear, move forward
    if distance_center > threshold:
        return 'MOVE_FORWARD'
    
    #Center blocked - choose direction with more space
    if distance_left > threshold and distance_left >= distance_right:
        return 'TURN_LEFT'
    elif distance_right > threshold:
        return 'TURN_RIGHT'
    else:
        #All sensors blocked - generic obstacle avoidance
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
        target: Dict {'x', 'y'} - target position
    
    Returns:
        str: 'MOVE_FORWARD', 'TURN_LEFT', or 'TURN_RIGHT'
    """
    robot_x = robot_pos.get('x', 0.0)
    robot_y = robot_pos.get('y', 0.0)
    robot_theta = robot_pos.get('theta', 0.0)
    
    #Calculate angle to target
    dx = target['x'] - robot_x
    dy = target['y'] - robot_y
    target_angle = math.atan2(dy, dx)
    
    #Calculate angle difference (normalize to -π to π)
    angle_diff = target_angle - robot_theta
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    
    #Decide action based on angle difference
    # Tolerance increased to prevent jitter (Hysteresis)
    angle_threshold = 0.2  # ~11 degrees
    
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
        
        # Read access for distance sensors and home position
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_center", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
        self.bb.register_key("home_position", access=py_trees.common.Access.READ)
    
    def update(self):
        battery = self.bb.get("battery")
        
        # Read distance sensors
        distance_left = self.bb.get("distance_left") or 999.0
        distance_center = self.bb.get("distance_center") or 999.0
        distance_right = self.bb.get("distance_right") or 999.0
        
        # Set goal to charging station (spawn platform - saved at startup)
        home_pos = self.bb.get("home_position") or HOME_POSITION_DEFAULT
        goal_pose = {
            'x': home_pos['x'],
            'y': home_pos['y'],
            'theta': home_pos['theta']
        }
        self.bb.set("goal_pose", goal_pose)
        
        # DEBUG: Log ritorno alla base
        print(f"[DEBUG][GoCharge] Batteria scarica! Ritorno alla piattaforma ({home_pos['x']:.2f}, {home_pos['y']:.2f})")
        
        # Calculate optimal direction based on 3 sensors
        action = calculate_best_direction(distance_left, distance_center, distance_right)
        self.bb.set("plan_action", action)
        
        # Simulate charging (random charge amount)
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
        self._last_logged_target = None  # Evita spam di log

    def update(self):
        # 1. Check if we already have a valid pending target (Persistence)
        current = self.bb.get("current_target")
        visited = self.bb.get("visited_targets") or []
        
        if current and current['name'] not in visited:
             # Keep current target until visited
             return Status.SUCCESS
             
        robot_pos_raw = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
        # Apply odometry correction if available
        odom_correction = self.bb.get("odom_correction") or {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0}
        robot_pos = {
            'x': robot_pos_raw.get('x', 0.0) + odom_correction.get('dx', 0.0),
            'y': robot_pos_raw.get('y', 0.0) + odom_correction.get('dy', 0.0),
            'theta': robot_pos_raw.get('theta', 0.0) + odom_correction.get('dtheta', 0.0)
        }
        
        # 2. Select NEW closest unvisited target
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
            # DEBUG: Log nuovo target selezionato
            if closest_name != self._last_logged_target:
                print(f"[DEBUG][CalculateTarget] Nuovo target: {closest_name.upper()} @ ({closest_data['x']:.2f}, {closest_data['y']:.2f}) | Distanza: {math.sqrt(closest_dist):.2f}m")
                print(f"[DEBUG][CalculateTarget] Posizione robot: ({robot_pos['x']:.2f}, {robot_pos['y']:.2f}) | Visitati: {visited}")
                self._last_logged_target = closest_name
            return Status.SUCCESS
        
        # DEBUG: Tutti i target visitati
        print(f"[DEBUG][CalculateTarget] Tutti i target visitati: {visited}")
        return Status.FAILURE

class AtTarget(py_trees.behaviour.Behaviour):
    """
    Check if robot has arrived at target and center on color.
    
    Logic:
    1. Check if near target (sensors detecting wall)
    2. CENTER on color: rotate slowly to maximize visible color area
    3. When area starts decreasing, STOP and check color
    4. If valve → SUCCESS, else mark visited and continue
    """
    def __init__(self):
        super().__init__(name="AtTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("current_target", access=py_trees.common.Access.WRITE)
        self.bb.register_key("detected_color", access=py_trees.common.Access.READ)
        self.bb.register_key("color_area", access=py_trees.common.Access.READ)
        self.bb.register_key("visited_targets", access=py_trees.common.Access.WRITE)
        self.bb.register_key("odom_correction", access=py_trees.common.Access.WRITE)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_center", access=py_trees.common.Access.READ)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("found", access=py_trees.common.Access.WRITE)
        
        # Internal state for centering behavior
        self.centering_state = 'APPROACHING'  # APPROACHING, CENTERING, DONE
        self.max_area_seen = 0
        self.area_history = []  # Track last N area readings
        self.rotation_direction = 'LEFT'  # Start by rotating left
        self.stable_count = 0  # Count of stable readings

    def initialise(self):
        """Reset centering state when behavior starts."""
        self.centering_state = 'APPROACHING'
        self.max_area_seen = 0
        self.area_history = []
        self.rotation_direction = 'LEFT'
        self.stable_count = 0

    def update(self):
        target = self.bb.get("current_target")
        if not target:
            return Status.FAILURE
        
        # Read distance sensors
        dist_left = self.bb.get("distance_left")
        dist_right = self.bb.get("distance_right")
        dist_center = self.bb.get("distance_center")
        
        # Read color area
        current_area = self.bb.get("color_area") or 0
        
        # Step 1: Check if we're near target (center sensor detects wall)
        near_wall_threshold = 0.8  # meters
        if dist_center is None or dist_center > near_wall_threshold:
            # Not near target yet
            self.centering_state = 'APPROACHING'
            return Status.FAILURE
        
        # DEBUG: Vicino al muro
        if self.centering_state == 'APPROACHING':
            print(f"[DEBUG][AtTarget] Vicino al muro! Distanza centro: {dist_center:.2f}m - Inizio CENTERING")
            self.centering_state = 'CENTERING'
            self.max_area_seen = current_area
            self.area_history = [current_area]
        
        # Step 2: CENTERING - rotate to maximize color area
        if self.centering_state == 'CENTERING':
            # Add current area to history
            self.area_history.append(current_area)
            if len(self.area_history) > 5:
                self.area_history.pop(0)
            
            # Calculate average of recent readings for stability
            avg_area = sum(self.area_history) / len(self.area_history)
            
            # Update max area seen
            if current_area > self.max_area_seen:
                self.max_area_seen = current_area
                self.stable_count = 0
            
            # Check if area is stable (not increasing anymore)
            area_decrease_threshold = 0.9  # 90% of max
            if avg_area < self.max_area_seen * area_decrease_threshold and self.max_area_seen > 1000:
                # Area is decreasing - we passed the optimal point
                # Reverse direction briefly to go back to max
                self.stable_count += 1
                
                if self.stable_count >= 3:
                    # Centered! Stop here
                    print(f"[DEBUG][AtTarget] CENTRATO! Area max: {self.max_area_seen}, Area attuale: {current_area:.0f}")
                    self.centering_state = 'DONE'
            else:
                # Keep rotating slowly
                self.stable_count = 0
                if self.rotation_direction == 'LEFT':
                    self.bb.set("plan_action", "TURN_LEFT")
                else:
                    self.bb.set("plan_action", "TURN_RIGHT")
                
                if len(self.area_history) >= 3 and self.area_history[-1] < self.area_history[-3]:
                    # Area decreasing - reverse direction
                    self.rotation_direction = 'RIGHT' if self.rotation_direction == 'LEFT' else 'LEFT'
                    print(f"[DEBUG][AtTarget] Inversione direzione: {self.rotation_direction}")
                    
                print(f"[DEBUG][AtTarget] Centering... Area: {current_area:.0f} (max: {self.max_area_seen:.0f}) | Dir: {self.rotation_direction}")
                return Status.RUNNING
        
        # Step 3: DONE - Stop and check color
        self.bb.set("plan_action", "STOP")
        
        detected_color = self.bb.get("detected_color")
        target_name = target.get("name")
        
        print(f"[DEBUG][AtTarget] Allineato con area massima! Area: {current_area:.0f}")
        
        # Calculate odometry correction offset
        robot_pos = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        odom_correction = {
            'dx': target['x'] - robot_pos.get('x', 0.0),
            'dy': target['y'] - robot_pos.get('y', 0.0),
            'dtheta': target['theta'] - robot_pos.get('theta', 0.0)
        }
        self.bb.set("odom_correction", odom_correction)
        print(f"[DEBUG][AtTarget] Correzione odometria: dx={odom_correction['dx']:.2f}m, dy={odom_correction['dy']:.2f}m, dθ={math.degrees(odom_correction['dtheta']):.1f}°")
        
        # Mark target as visited
        visited = self.bb.get("visited_targets") or []
        if target_name not in visited:
            visited.append(target_name)
            self.bb.set("visited_targets", visited)
            print(f"[DEBUG][AtTarget] Target {target_name.upper()} raggiunto e marcato come visitato!")
        
        # Step 4: Check if this is the valve (red color)
        if detected_color == "red":
            # FOUND THE VALVE!
            print(f"[DEBUG][AtTarget] VALVOLA TROVATA! Colore rilevato: {detected_color}")
            self.bb.set("found", "valve")
            return Status.SUCCESS
        
        # Not the valve - clear current_target so CalculateTarget picks the next one
        self.bb.set("current_target", None)
        print(f"[DEBUG][AtTarget] Colore rilevato: {detected_color or 'nessuno'} - Non è la valvola, passo al prossimo target...")
        return Status.SUCCESS

class InitialRetreat(py_trees.behaviour.Behaviour):
    """
    Step 1: Move backward from the platform at startup.
    Saves initial spawn position before retreating.
    Runs only once until completion.
    """
    def __init__(self):
        super().__init__(name="InitialRetreat")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("startup_complete", access=py_trees.common.Access.WRITE)
        self.bb.register_key("startup_complete", access=py_trees.common.Access.READ)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("home_position", access=py_trees.common.Access.WRITE)
        self.start_time = None
        self.duration = 4.0 # Seconds to retreat
        self._logged_start = False
        self._home_saved = False
    
    def update(self):
        if self.bb.get("startup_complete"):
            return Status.SUCCESS
        
        # FIRST: Save initial position as home BEFORE moving
        if not self._home_saved:
            robot_pos = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
            home_position = {
                'x': robot_pos['x'],
                'y': robot_pos['y'],
                'theta': robot_pos['theta']
            }
            self.bb.set("home_position", home_position)
            self._home_saved = True
            print(f"[DEBUG][InitialRetreat] HOME salvata: ({home_position['x']:.2f}, {home_position['y']:.2f}) theta={math.degrees(home_position['theta']):.1f}°")
            
        if self.start_time is None:
            self.start_time = time.time()
            # DEBUG: Log inizio retreat
            print(f"[DEBUG][InitialRetreat] Inizio arretramento dalla piattaforma ({self.duration}s)...")
            
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            self.bb.set("plan_action", "MOVE_BACKWARD")
            # DEBUG: Log progresso ogni secondo
            if int(elapsed) > int(elapsed - 0.1) and not self._logged_start:
                print(f"[DEBUG][InitialRetreat] Arretramento... {elapsed:.1f}s / {self.duration}s")
            return Status.RUNNING
        else:
            self.bb.set("plan_action", "STOP")
            self.bb.set("startup_complete", True)
            # DEBUG: Log completamento
            print(f"[DEBUG][InitialRetreat] Arretramento completato! Inizio navigazione...")
            return Status.SUCCESS

class MoveToTarget(py_trees.behaviour.Behaviour):
    """
    Advanced Navigation with Hysteresis & Obstacle Recovery.
    
    Logic:
    1. Check Ultrasonics (< 0.5m)
       - If blocked: Enter AVOIDANCE mode (turn to open space).
    2. Check AVOIDANCE recovery
       - If was avoiding and now clear: Move Forward blindly (1-2s) to pass obstacle.
    3. Navigation (Target)
       - If clear: Calculate angle error.
       - Only change direction if error is large (> threshold).
       - Maintain current direction otherwise (Hysteresis).
    """
    def __init__(self):
        super().__init__(name="MoveToTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        
        # Write access
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("goal_pose", access=py_trees.common.Access.WRITE)
        
        # Read access
        self.bb.register_key("current_target", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_center", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.bb.register_key("odom_correction", access=py_trees.common.Access.READ)
        
        # Internal State
        self.avoiding = False
        self.recovery_steps = 0
        self.last_action = "STOP"
        self._debug_tick = 0  # Contatore per logging periodico

    def update(self):
        # 1. Get Target (Persistent)
        target = self.bb.get("current_target")
        if not target:
            self.bb.set("plan_action", "STOP")
            return Status.FAILURE # Should trigger CalculateTarget

        # 2. Update Blackboard Goal (for logging/debug)
        self.bb.set("goal_pose", target)

        # 3. Read Sensors & Pos
        d_left = self.bb.get("distance_left") or 999.0
        d_center = self.bb.get("distance_center") or 999.0
        d_right = self.bb.get("distance_right") or 999.0
        robot_pos_raw = self.bb.get("robot_position") or {'x':0, 'y':0, 'theta':0}
        
        # Apply odometry correction if available
        odom_correction = self.bb.get("odom_correction") or {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0}
        robot_pos = {
            'x': robot_pos_raw.get('x', 0.0) + odom_correction.get('dx', 0.0),
            'y': robot_pos_raw.get('y', 0.0) + odom_correction.get('dy', 0.0),
            'theta': robot_pos_raw.get('theta', 0.0) + odom_correction.get('dtheta', 0.0)
        }
        
        # Thresholds
        OBSTACLE_DIST = 0.5
        
        # --- LOGIC START ---
        
        # A. OBSTACLE DETECTION
        is_blocked = (d_center < OBSTACLE_DIST or d_left < OBSTACLE_DIST or d_right < OBSTACLE_DIST)
        
        if is_blocked:
            # DEBUG: Log ostacolo rilevato
            if not self.avoiding:
                print(f"[DEBUG][MoveToTarget] OSTACOLO RILEVATO! Distanze: L={d_left:.2f}m C={d_center:.2f}m R={d_right:.2f}m")
            
            self.avoiding = True
            self.recovery_steps = 0 # Reset recovery
            
            # Escape Logic
            if d_left < OBSTACLE_DIST and d_center < OBSTACLE_DIST and d_right < OBSTACLE_DIST:
                 action = "TURN_RIGHT" # Trapped -> Spin
                 if self._debug_tick % 10 == 0:
                     print(f"[DEBUG][MoveToTarget] INTRAPPOLATO! Rotazione a destra...")
            elif d_left > d_right:
                action = "TURN_LEFT" # Left is open
                if self._debug_tick % 10 == 0:
                    print(f"[DEBUG][MoveToTarget] Evitamento: giro a SINISTRA (più spazio)")
            else:
                action = "TURN_RIGHT" # Right is open (or both blocked center)
                if self._debug_tick % 10 == 0:
                    print(f"[DEBUG][MoveToTarget] Evitamento: giro a DESTRA")
                
            self.last_action = action
            self.bb.set("plan_action", action)
            self._debug_tick += 1
            return Status.RUNNING

        # B. RECOVERY (Post-Avoidance)
        if self.avoiding:
            # Obstacle is cleared, but we need to move away from it before turning back
            if self.recovery_steps < 15: # ~1.5 seconds at 10Hz
                if self.recovery_steps == 0:
                    print(f"[DEBUG][MoveToTarget] Recupero post-ostacolo: avanzo per superare...")
                action = "MOVE_FORWARD"
                self.recovery_steps += 1
                self.bb.set("plan_action", action)
                return Status.RUNNING
            else:
                # Recovery done, recalculate path from new position
                self.avoiding = False
                
                # Calculate new distance and angle from current position
                new_dx = target['x'] - robot_pos.get('x', 0.0)
                new_dy = target['y'] - robot_pos.get('y', 0.0)
                new_distance = math.sqrt(new_dx**2 + new_dy**2)
                new_angle = math.atan2(new_dy, new_dx)
                angle_error = math.degrees(new_angle - robot_pos.get('theta', 0.0))
                
                print(f"[DEBUG][MoveToTarget] Recupero completato! RICALCOLO PERCORSO...")
                print(f"[DEBUG][MoveToTarget] Nuova posizione: ({robot_pos.get('x', 0.0):.2f}, {robot_pos.get('y', 0.0):.2f})")
                print(f"[DEBUG][MoveToTarget] Target {target['name'].upper()}: distanza={new_distance:.2f}m, errore angolare={angle_error:.1f}°")
        
        # C. NAVIGATION (Hysteresis)
        # Calculate angle error to target
        robot_x = robot_pos.get('x', 0.0)
        robot_y = robot_pos.get('y', 0.0)
        robot_theta = robot_pos.get('theta', 0.0)
        
        dx = target['x'] - robot_x
        dy = target['y'] - robot_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        angle_diff = target_angle - robot_theta
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Hysteresis thresholds
        HYSTERESIS_THRESHOLD = 0.4  # ~23 degrees - switch action only if error is big
        ALIGNMENT_THRESHOLD = 0.2   # ~11 degrees - considered aligned with target
        
        # Decision logic with hysteresis
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
                # Was doing something else (avoidance) - pick appropriate
                action = 'MOVE_FORWARD'
        
        # DEBUG: Log navigazione periodico (ogni 20 tick = 2 secondi)
        if self._debug_tick % 20 == 0:
            angle_deg = math.degrees(angle_diff)
            theta_deg = math.degrees(robot_theta)
            print(f"[DEBUG][MoveToTarget] NAV → {target['name'].upper()} | Dist: {distance_to_target:.2f}m | Errore ang: {angle_deg:.1f}° | Azione: {action}")
            print(f"[DEBUG][MoveToTarget] Pos: ({robot_x:.2f}, {robot_y:.2f}) θ={theta_deg:.1f}° | Sensori: L={d_left:.2f} C={d_center:.2f} R={d_right:.2f}")
        
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

#Actions & Signals
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
    """
    Smart avoidance for person - turn to the opposite side of where person is detected.
    Uses detection_zone from camera to decide direction.
    """
    def __init__(self):
        super().__init__(name="GoAroundP")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("detection_zone", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_left", access=py_trees.common.Access.READ)
        self.bb.register_key("distance_right", access=py_trees.common.Access.READ)
    
    def update(self):
        zone = self.bb.get("detection_zone")
        dist_left = self.bb.get("distance_left") or 999.0
        dist_right = self.bb.get("distance_right") or 999.0
        
        # Turn opposite to where person is detected
        if zone == 'left':
            # Person on left → turn right
            action = 'TURN_RIGHT'
        elif zone == 'right':
            # Person on right → turn left
            action = 'TURN_LEFT'
        else:
            # Person in center → turn to more open side
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
        
        # Turn opposite to where obstacle is detected
        if zone == 'left':
            action = 'TURN_RIGHT'
        elif zone == 'right':
            action = 'TURN_LEFT'
        else:
            # Obstacle in center → turn to more open side
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
        
        # DEBUG: Missione completata!
        print(f"")
        print(f"[DEBUG][ActiveValve] ══════════════════════════════════════")
        print(f"[DEBUG][ActiveValve] MISSIONE COMPLETATA! VALVOLA ATTIVATA!")
        print(f"[DEBUG][ActiveValve] ══════════════════════════════════════")
        print(f"")
        
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
        self.bb.register_key("home_position", access=py_trees.common.Access.READ)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("goal_pose", access=py_trees.common.Access.WRITE)
    
    def update(self):
        # Home position = spawn platform (saved at startup by InitialRetreat)
        home_pos = self.bb.get("home_position") or HOME_POSITION_DEFAULT
        home_x = home_pos['x']
        home_y = home_pos['y']
        home_theta = home_pos['theta']
        
        # Get current robot position from odometry
        robot_pos = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        current_x = robot_pos.get('x', 0.0)
        current_y = robot_pos.get('y', 0.0)
        
        # Calculate Euclidean distance to home
        distance = ((current_x - home_x)**2 + (current_y - home_y)**2)**0.5
        
        # DEBUG: Log periodico navigazione verso casa
        if not hasattr(self, '_logged_going_home'):
            self._logged_going_home = True
            print(f"[DEBUG][GoHome] Ritorno alla piattaforma di spawn ({home_x:.2f}, {home_y:.2f}) theta={math.degrees(home_theta):.1f}°")
        
        # If close to home (< 0.3m), stop
        if distance < 0.3:
            self.bb.set("plan_action", "STOP")
            print(f"[DEBUG][GoHome] Arrivato alla piattaforma! Distanza: {distance:.2f}m")
            return Status.SUCCESS
        
        # Otherwise, set goal to home and continue moving
        goal_pose = {
            'x': home_x,
            'y': home_y,
            'theta': home_theta
        }
        self.bb.set("goal_pose", goal_pose)
        self.bb.set("plan_action", "MOVE_TO_GOAL")
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
        - GoHome
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
        RecognitionValve()
    ])
    
    #Retry target search until valve is found (-1 = infinite retries)
    target_loop = decorators.Retry("TargetLoop", child=target_search, num_failures=-1)
    
    #Main mission sequence
    main = Sequence("Main", memory=True, children=[
        InitialRetreat(),
        battery,
        target_loop,  # Loop until valve found
        ActiveValve(),
        GoHome()
    ])
    
    #Retry until mission success
    root = decorators.Retry("LoopUntilSuccess", child=main, num_failures=-1)
    
    return root

#ROS2 NODE
#Mapping from internal actions to Act module commands
ACTION_TO_COMMAND = {
    'MOVE_FORWARD': 'Front',
    'MOVE_BACKWARD': 'Back',
    'TURN_LEFT': 'Left',
    'TURN_RIGHT': 'Right',
    'MOVE_FRONT_LEFT': 'FrontLeft',
    'MOVE_FRONT_RIGHT': 'FrontRight',
    'STOP': 'Stop',
    'IDLE': 'Stop',
    'AVOID_OBSTACLE': 'Left',
    'ACTIVATE_VALVE': 'Stop',
    'MOVE_TO_GOAL': 'Front',
}

class PlanNode(Node):
    """
    Plan ROS2 Node - Strategic decision layer for Charlie Robot.
    
    Subscribes to:
        /sense/world_state - JSON with battery, odometry, obstacles, detections, detected_color
    
    Publishes to:
        /plan/command - String command for Act (Front, Left, Right, Stop)
        /plan/signals - JSON array of signals (PersonFound, ValveActivated)
    """
    
    def __init__(self):
        super().__init__('plan_node')
        self.get_logger().info('=== Plan Node Starting ===')
        
        # Simulation readiness flag
        self.simulation_ready = False
        self.tick_timer = None
        
        #Build Behavior Tree
        self.tree = build_tree()
        self.bb = py_trees.blackboard.Client(name="PlanNode")
        
        #Register all blackboard keys with write access
        blackboard_keys = [
            'battery', 'obstacles', 'detections', 'targets',
            'current_target', 'visited_targets', 'found', 'signals',
            'plan_action', 'goal_pose', 'mission_complete',
            'detected_color', 'color_area', 'odom_correction', 'detection_zone',
            'distance_left', 'distance_center', 'distance_right',
            'robot_position', 'startup_complete', 'home_position'
        ]
        for key in blackboard_keys:
            self.bb.register_key(key, access=py_trees.common.Access.WRITE)
        
        self._init_blackboard()
        self.tree.setup_with_descendants()
        
        #Subscriptions to Sense topics
        self.create_subscription(
            Range, '/sense/proximity/front',
            lambda msg: self._proximity_cb(msg, 'center'), 10
        )
        self.create_subscription(
            Range, '/sense/proximity/front_left',
            lambda msg: self._proximity_cb(msg, 'left'), 10
        )
        self.create_subscription(
            Range, '/sense/proximity/front_right',
            lambda msg: self._proximity_cb(msg, 'right'), 10
        )
        self.create_subscription(Pose2D, '/sense/odometry', self._odom_cb, 10)
        self.create_subscription(String, '/sense/detection', self._detection_cb, 10)
        
        #Publishers to Act module
        self.cmd_pub = self.create_publisher(String, '/plan/command', 10)
        self.signals_pub = self.create_publisher(String, '/plan/signals', 10)
        
        # Wait for simulation to be ready before starting BT tick
        # Use a timer to poll for /robot_description topic availability
        self.get_logger().info('Waiting for simulation to be ready (checking /robot_description topic)...')
        self.create_timer(1.0, self._check_simulation_ready)
    
    def _check_simulation_ready(self):
        """
        Check if simulation is ready by verifying /robot_description topic exists.
        Once ready, wait additional time for robot spawn, then start the Behavior Tree tick timer.
        """
        if self.simulation_ready:
            return  # Already ready, nothing to do
        
        # Get list of available topics
        topic_names_and_types = self.get_topic_names_and_types()
        topic_names = [name for name, _ in topic_names_and_types]
        
        if '/robot_description' in topic_names:
            self.simulation_ready = True
            self.get_logger().info('✓ /robot_description topic found!')
            self.get_logger().info('Waiting 5 seconds for robot to complete spawn...')
            
            # Schedule the BT tick timer to start after 5 seconds delay
            # This gives time for the robot to be fully spawned in Gazebo
            self.create_timer(5.0, self._start_behavior_tree, callback_group=None)
        else:
            self.get_logger().info('Waiting for simulation... /robot_description not yet available')
    
    def _start_behavior_tree(self):
        """
        Start the Behavior Tree tick timer after spawn delay.
        Called once after robot is fully ready.
        """
        if self.tick_timer is not None:
            return  # Already started, nothing to do
        
        self.get_logger().info('Starting Behavior Tree tick timer (10 Hz)...')
        self.tick_timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('Plan Node ready - sending commands to robot')
    
    def _init_blackboard(self):
        """Initialize blackboard with default values."""
        #Base data
        self.bb.set("battery", 100.0)  #TODO: Sense doesn't publish battery
        self.bb.set("obstacles", [])
        self.bb.set("detections", {})
        
        #Target navigation
        self.bb.set("current_target", None)
        self.bb.set("visited_targets", [])
        
        #Object detection & signals
        self.bb.set("found", None)
        self.bb.set("signals", [])
        
        #Commands to Act module
        self.bb.set("plan_action", "IDLE")
        self.bb.set("goal_pose", None)
        self.bb.set("mission_complete", False)
        
        #Sensor data from Sense module
        self.bb.set("detected_color", None)
        self.bb.set("color_area", 0)
        self.bb.set("detection_zone", None)
        self.bb.set("odom_correction", {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0})
        self.bb.set("distance_left", 999.0)
        self.bb.set("distance_center", 999.0)
        self.bb.set("distance_right", 999.0)
        self.bb.set("robot_position", {'x': 0.0, 'y': 0.0, 'theta': 0.0})
        self.bb.set("startup_complete", False)
    
    def _proximity_cb(self, msg, sensor):
        """
        Callback for proximity sensors (Range messages).
        
        Args:
            msg: Range message from Sense
            sensor: 'left', 'center', or 'right'
        """
        distance = msg.range if msg.range >= 0 else 999.0
        
        if sensor == 'center':
            self.bb.set("distance_center", distance)
        elif sensor == 'left':
            self.bb.set("distance_left", distance)
        elif sensor == 'right':
            self.bb.set("distance_right", distance)
    
    def _odom_cb(self, msg):
        """
        Callback for odometry (Pose2D message).
        
        Args:
            msg: Pose2D message with x, y, theta
        """
        self.bb.set("robot_position", {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.theta
        })
    
    def _detection_cb(self, msg):
        """
        Callback for detection events (String JSON).
        
        Expected format:
        {
            "type": "person" | "target" | "obstacle" | "none",
            "color": "red" | "green" | "blue" | null,
            "bbox_area": int,
            "zone": "left" | "center" | "right" | null,
            "estimated_distance": float,
            "proximity_distance": float,
            "confidence": float
        }
        """
        try:
            det = json.loads(msg.data)
            
            #Set detected_color from detection
            self.bb.set("detected_color", det.get('color'))
            
            #Set color_area for centering behavior (maximize visible area)
            self.bb.set("color_area", det.get('bbox_area', 0))
            
            #Set detection_zone for smart avoidance
            self.bb.set("detection_zone", det.get('zone'))
            
            #Build detections dict for compatibility
            detections = {}
            if det['type'] == 'person':
                detections['person'] = True
            elif det['type'] == 'obstacle':
                detections['obstacle'] = True
            
            self.bb.set("detections", detections)
            
            #Set found based on detection type and color
            if det['type'] == 'person':
                self.bb.set("found", "person")
            elif det.get('color') == 'red':
                #Red color = valve
                self.bb.set("found", "valve")
            elif det['type'] == 'obstacle':
                self.bb.set("found", "obstacle")
            else:
                self.bb.set("found", None)
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse detection: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in _detection_cb: {e}")
    
    def _tick(self):
        """
        Timer callback - tick Behavior Tree and publish outputs.
        Called at 10 Hz (every 0.1 seconds).
        """
        self.tree.tick_once()
        
        #Convert internal action to Act command
        action = self.bb.get("plan_action") or "STOP"
        command = ACTION_TO_COMMAND.get(action, "Stop")

        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        
        #Publish signals if any
        signals = self.bb.get("signals") or []
        if signals:
            signals_msg = String()
            signals_msg.data = json.dumps(signals)
            self.signals_pub.publish(signals_msg)
            self.bb.set("signals", [])

#entry point
def main(args=None):
    """Main entry point for Plan Node."""
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 not available. Install rclpy to run this node.")
        return
    
    rclpy.init(args=args)
    node = PlanNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
