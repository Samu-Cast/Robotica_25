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
    'green': {'x': -3, 'y': -6.0, 'theta': -1.57}, #la x è 
    'blue': {'x': 0.35, 'y': -4.0, 'theta': 0.0},
    'red': {'x': -7, 'y': -2.5, 'theta': 3.14},  # This is actually the valve, but robot doesn't know
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
def calculate_best_direction(distance_left, distance_center, distance_right, threshold=0.5):
    """
    Calculate the optimal movement direction based on 3 distance sensors.
    
    FIRST evaluates ALL sensors, THEN decides the best action.
    This ensures the robot is aware of obstacles on all sides before moving.
    
    Args:
        distance_left: Distance reading from left sensor (meters)
        distance_center: Distance reading from center sensor (meters)  
        distance_right: Distance reading from right sensor (meters)
        threshold: Minimum safe distance to consider path clear (default 0.5m)
    
    Returns:
        str: One of 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT', or 'AVOID_OBSTACLE'
    """
    #Evaluate ALL sensors FIRST 
    left_blocked = distance_left < threshold
    center_blocked = distance_center < threshold
    right_blocked = distance_right < threshold
    
    #Decide based on complete awareness
    if left_blocked and center_blocked and right_blocked:
        #All directions blocked - need to escape (spin)
        return 'AVOID_OBSTACLE'
    
    if not center_blocked:
        #Center is clear - but check if sides are too close for safety
        if left_blocked and not right_blocked:
            #Something on left, veer right while moving forward
            return 'TURN_RIGHT'
        elif right_blocked and not left_blocked:
            #Something on right, veer left while moving forward
            return 'TURN_LEFT'
        else:
            #Center clear and no side obstacles
            return 'MOVE_FORWARD'
    else:
        #Center is blocked - choose the side with more space
        if distance_left >= distance_right:
            return 'TURN_LEFT'
        else:
            return 'TURN_RIGHT'


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
        
        #DEBUG: Log ritorno alla base
        print(f"[DEBUG][GoCharge] Batteria scarica! Ritorno alla piattaforma ({home_pos['x']:.2f}, {home_pos['y']:.2f})")
        
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
        self.bb.register_key("bumper_event", access=py_trees.common.Access.READ)
        self.bb.register_key("detected_color", access=py_trees.common.Access.WRITE)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self._last_logged_target = None  #Evita spam di log

    def update(self):
        
        print(f"[DEBUG]                                              [CalculateTarget] ")

        #1. Check if we already have a valid pending target (Persistence)
        current = self.bb.get("current_target")
        visited = self.bb.get("visited_targets") or []
        
        if current and current['name'] not in visited:
             #Keep current target until visited
             print(f"IL TARGET CORRENTE è {current['name']}")
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
            print(f"Sto cambiando target verso {closest_name} ")
            self.bb.set("current_target", {
                'name': closest_name,
                'x': closest_data['x'],
                'y': closest_data['y'],
                'theta': closest_data['theta']
            })
            #DEBUG: Log nuovo target selezionato
            if closest_name != self._last_logged_target:
                print(f"[DEBUG][CalculateTarget] Nuovo target: {closest_name.upper()} @ ({closest_data['x']:.2f}, {closest_data['y']:.2f}) | Distanza: {math.sqrt(closest_dist):.2f}m")
                print(f"[DEBUG][CalculateTarget] Posizione robot: ({robot_pos['x']:.2f}, {robot_pos['y']:.2f}) | Visitati: {visited}")
                self._last_logged_target = closest_name
            return Status.SUCCESS
        
        #DEBUG: Tutti i target visitati
        print(f"[DEBUG][CalculateTarget] Tutti i target visitati: {visited}")
        return Status.FAILURE


class AtTarget(py_trees.behaviour.Behaviour):
    """
        Check if robot has arrived at the target.
        SBAGLIATOOO
        Complete Flow:
        1. Color Detection (< 1m): Center the color in camera
           - If left/right: turn until centered
           - Once centered: go straight (CHARGE_COLOR)
        2. Collision Detection (bumper=True):
           - Check if valve is red (SUCCESS if found)
           - If NOT red: Enter backup phase
        3. Backup Phase (if not valve):
           - Reverse for 6 seconds (BACK command)
           - Mark target as visited
           - Pass to next target
    """
    COLOR_DETECTION_DISTANCE = 1.0  # Distance threshold for color detection (meters)
    BACKUP_DURATION = 8.0  # Backup duration in seconds
    
    def __init__(self):
        super().__init__(name="AtTarget")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("current_target", access=py_trees.common.Access.WRITE)
        self.bb.register_key("detected_color", access=py_trees.common.Access.READ)
        self.bb.register_key("visited_targets", access=py_trees.common.Access.WRITE)
        self.bb.register_key("robot_position", access=py_trees.common.Access.WRITE)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("found", access=py_trees.common.Access.WRITE)
        self.bb.register_key("bumper_event", access=py_trees.common.Access.WRITE)
        self.bb.register_key("bumper", access=py_trees.common.Access.WRITE)
        self.bb.register_key("odom_correction", access=py_trees.common.Access.WRITE)
        self.bb.register_key("detection_zone", access=py_trees.common.Access.READ)
        
        self.start_time = None #per contare il tempo di retreat
        self.duration = 15.0  # Seconds to retreat
        self._logged_start = False #serve per
        self._backup_start_time = None  # For backup phase timing

        # Internal state
        self._color_detected_logged = False 
        self._centering_complete = False
        self._backing_up = False
        self._backup_start_time = None  

    def update(self):
        target = self.bb.get("current_target")
        print(f"[DEBUG]                                              [AT TARGET] Verso {target['name']}")
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
        robot_theta = robot_pos.get('theta', 0.0)
        
        # Calculate distance to target (for reference/logging)
        dx = target['x'] - robot_x
        dy = target['y'] - robot_y
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        bumper1 = self.bb.get("bumper")

        if distance_to_target <= self.COLOR_DETECTION_DISTANCE and bumper1==True:
            bumper_pressed = self.bb.get("bumper_event")
        else:
            bumper_pressed = False
        
        print(f"Bumper1 è {bumper1} -----------   il bumper_pressed è {bumper_pressed}")

        detected_color = self.bb.get("detected_color") #colore rilevato dalla cameRA     
        print(f"[DEBUG][AtTarget] Colore visto dalla fotocamera è {detected_color}  Il bumper è {bumper_pressed}")
        target_name = target.get("name")
        detection_zone = self.bb.get("detection_zone")  # 'left', 'center', or 'right'
        

        
        # ============================================================================
        # PHASE 1: COLOR DETECTION & CENTERING (< 1 meter)
        # ============================================================================
        if detected_color and distance_to_target <= self.COLOR_DETECTION_DISTANCE and bumper_pressed == False:
            if not self._color_detected_logged:
                print(f"[DEBUG][AtTarget] === COLOR DETECTED === {detected_color.upper()} at {target_name.upper()} (dist: {distance_to_target:.2f}m)")
                self._color_detected_logged = True 
                self._centering_complete = False
            
            # PHASE 1a: Center color in camera frame
            if detection_zone and detection_zone != 'center':
                # Color not centered - rotate to center it
                if detection_zone == 'left':
                    print(f"[DEBUG][AtTarget] Color LEFT → TURN_LEFT to center")
                    self.bb.set("plan_action", "TURN_LEFT")
                elif detection_zone == 'right':
                    print(f"[DEBUG][AtTarget] Color RIGHT → TURN_RIGHT to center")
                    self.bb.set("plan_action", "TURN_RIGHT")
                
                return Status.RUNNING
            else:
                # PHASE 1b: Color is centered - charge toward it
                if not self._centering_complete:
                    print(f"[DEBUG][AtTarget] Color Centrato -- Verso il muro...")
                    self._centering_complete = True
                
                self.bb.set("plan_action", "CHARGE_COLOR")
                
                return Status.RUNNING
        
        # Reset centering when color is lost or too far
        if not detected_color or distance_to_target > self.COLOR_DETECTION_DISTANCE:
            self._color_detected_logged = False
            self._centering_complete = False
        
        if bumper_pressed and distance_to_target <= self.COLOR_DETECTION_DISTANCE:
            print(f"[DEBUG][AtTarget] 💥 COLLISIONEEEEEEE! Bumper attivato a {target_name.upper()}")
            print(f"[DEBUG][AtTarget] Posizione: ({robot_x:.2f}, {robot_y:.2f}) | Colore: {detected_color or 'NONE'}")           
            
            if detected_color == "red":
                self.bb.set("found", "valve")
            
            
            # Mark target as visited
            visited = self.bb.get("visited_targets") or []
            if target_name not in visited:
                visited.append(target_name)
                self.bb.set("visited_targets", visited)
                print(f"[DEBUG][AtTarget] Target {target_name.upper()} marcato come VISITED")

            self.bb.set("plan_action", "STOP")
                

            # Reset state for next target
            self.bb.set("current_target", None)
            self.bb.set("bumper_event", False)  # Reset bumper event for next target
            print(f"Dopo la collisione, resetto il bumper e il target corrente è {self.bb.get('bumper_event')}")    
            self._backing_up = False
            self._backup_start_time = None
            self._color_detected_logged = False
            self._centering_complete = False
            print(f"Sto facendo SUCCESS finito AtTarget, dovrei anadare in SecondoRetreat")
            self.bb.set("plan_action", "STOP")

            print(f"[DEBUG][AtTarget] Correcting odometry after visit to {target_name.upper()}")
            
            print(f"[DEBUG][AtTarget] ✅✅✅ CORDINATE ROBOT {self.bb.get('robot_position')} ")
            print(f"[DEBUG][AtTarget] ✅✅✅ CORDINATE TARGET {target} ")
            print(f"[DEBUG][AtTarget] ✅✅✅ ODOMETRIA {self.bb.get('odom_correction')} ")
            
            print(f"[DEBUG][AtTarget] ❌❌❌ CORDINATE ROBOT {self.bb.get('robot_position')} ")
            print(f"[DEBUG][AtTarget] ❌❌❌ CORDINATE TARGET {target} ")
            print(f"[DEBUG][AtTarget] ❌❌❌ ODOMETRIA {self.bb.get('odom_correction')} ")
            
            return Status.SUCCESS
        
        # No bumper - not at target yet
        return Status.FAILURE


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
        self.duration = 20.0  # Seconds to retreat
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


class SecondoRetreat(py_trees.behaviour.Behaviour):
    """
   secondo indietro quando sta in un target e non è il valvola
    """
    def __init__(self):
        super().__init__(name="SecondoRetreat")
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key("plan_action", access=py_trees.common.Access.WRITE)
        self.bb.register_key("bumper_event", access=py_trees.common.Access.WRITE)
        self.bb.register_key("current_target", access=py_trees.common.Access.WRITE)
        self.bb.register_key("robot_position", access=py_trees.common.Access.READ)
        self.start_time = None #per contare il tempo di retreat
        self.duration = 15.0  # Seconds to retreat
        self._logged_start = False #serve per
        
    
    def update(self):
        print(f"[DEBUG]                           [SecondoRetreat]" )

        self.bb.set("bumper_event", False)
        self.bb.set("plan_action", "STOP")
        self.bb.set("current_target", None)
                
        self._backing_up = False
        self._backup_start_time = None
        self._color_detected_logged = False
        self._centering_complete = False
        
        if self.start_time is None:
            self.start_time = time.time()
            # DEBUG: Log inizio retreat
            print(f"[DEBUG][SecondoRetreat] Inizio arretramento dal target ({self.duration}s)...")
            
        elapsed = time.time() - self.start_time #time.time - self.start_time mi
        print(f"tempo trascorso: {elapsed}, durata: {self.duration}") 
        if elapsed < self.duration: #arretro se 
            print(f"STO ARRETRANDO ACT= BACK")
            self.bb.set("plan_action", "MOVE_BACKWARD")
            # DEBUG: Log progresso ogni secondo
            print(f"[DEBUG][SecondoRetreat] Arretramento... {elapsed:.1f}s / {self.duration}s")
            return Status.RUNNING
        else:
            self.bb.set("plan_action", "STOP")
            # DEBUG: Log completamento
            print(f"[DEBUG][SecondoRetreat] Arretramento completato! Inizio prossimo Target...")
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
        
        #Internal State
        self.avoiding = False
        self.recovery_steps = 0
        self.last_action = "STOP"
        self._debug_tick = 0  #Contatore per logging periodico

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
        
        #Thresholds
        OBSTACLE_DIST = 0.5
        
        #CHECK FOR CHARGE_COLOR MODE (Ignore obstacles, go straight to colored wall)
        current_action = self.bb.get("plan_action")
        if current_action == "CHARGE_COLOR":
            #In charge mode - ignore all obstacles, go straight
            self.bb.set("plan_action", "MOVE_FORWARD")
            if self._debug_tick % 20 == 0:
                print(f"[DEBUG][MoveToTarget] MODALITÀ CARICA COLORE ATTIVA - Vado dritto verso {target['name'].upper()}!")
            self._debug_tick += 1
            return Status.RUNNING
        
        #LOGIC START
        
        #A. CAMERA DETECTION (person/obstacle from Sense module)
        found = self.bb.get("found")
        detection_zone = self.bb.get("detection_zone")
        detection_distance = self.bb.get("detection_distance") or 999.0
        detection_confidence = self.bb.get("detection_confidence") or 0.0
        
        #Only react to camera detections if:
        #1. It's a person or obstacle
        #2. It's close enough (< 2.0m estimated)
        #3. Confidence is high enough (> 0.5)
        #NOTE: We check even while avoiding - keep turning until obstacle is out of view
        CAMERA_AVOID_THRESHOLD = 0.5  #meters - react only when person is close
        
        obstacle_visible = (found in ['person', 'obstacle'] 
                           and detection_distance < CAMERA_AVOID_THRESHOLD 
                           and detection_confidence > 0.5)
        
        if obstacle_visible:
            #Obstacle is visible - keep turning to avoid it
            self.avoiding = True
            self.recovery_steps = 0  #Reset recovery (obstacle still visible)
            
            #Turn opposite to detection zone
            if detection_zone == 'left':
                action = 'TURN_RIGHT'
            elif detection_zone == 'right':
                action = 'TURN_LEFT'
            else:
                #Center or unknown - choose based on ultrasonic sensors
                action = 'TURN_LEFT' if d_left > d_right else 'TURN_RIGHT'
            
            if self._debug_tick % 5 == 0:
                print(f"[DEBUG][MoveToTarget] {found.upper()} visibile! Continuo a girare... Zona: {detection_zone} → {action}")
            self.bb.set("plan_action", action)
            self._debug_tick += 1
            return Status.RUNNING
        
        #B. ULTRASONIC OBSTACLE DETECTION
        is_blocked = (d_center < OBSTACLE_DIST or d_left < OBSTACLE_DIST or d_right < OBSTACLE_DIST)
        
        if is_blocked:
            self.avoiding = True
            self.recovery_steps = 0
            
            #Use calculate_best_direction for smart avoidance
            action = calculate_best_direction(d_left, d_center, d_right, OBSTACLE_DIST)
            
            if self._debug_tick % 10 == 0:
                print(f"[DEBUG][MoveToTarget] OSTACOLO ultrasuoni! L={d_left:.2f} C={d_center:.2f} R={d_right:.2f} → {action}")
            
            self.last_action = action
            self.bb.set("plan_action", action)
            self._debug_tick += 1
            return Status.RUNNING

        #C. RECOVERY (Post-Avoidance)
        if self.avoiding:
            #Obstacle is cleared, but we need to move away from it before turning back
            if self.recovery_steps < 3:  #~0.3 seconds - quick recovery then recalculate
                if self.recovery_steps == 0:
                    print(f"[DEBUG][MoveToTarget] Recupero post-ostacolo: avanzo per superare...")
                action = "MOVE_FORWARD"
                self.recovery_steps += 1
                self.bb.set("plan_action", action)
                return Status.RUNNING
            else:
                #Recovery done, recalculate path from new position
                self.avoiding = False
                
                #Calculate new distance and angle from current position
                new_dx = target['x'] - robot_pos.get('x', 0.0)
                new_dy = target['y'] - robot_pos.get('y', 0.0)
                new_distance = math.sqrt(new_dx**2 + new_dy**2)
                new_angle = math.atan2(new_dy, new_dx)
                angle_error = math.degrees(new_angle - robot_pos.get('theta', 0.0))
                
                print(f"[DEBUG][MoveToTarget] Recupero completato! RICALCOLO PERCORSO...")
                print(f"[DEBUG][MoveToTarget] Nuova posizione: ({robot_pos.get('x', 0.0):.2f}, {robot_pos.get('y', 0.0):.2f})")
                print(f"[DEBUG][MoveToTarget] Target {target['name'].upper()}: distanza={new_distance:.2f}m, errore angolare={angle_error:.1f}°")
        
        #D. NAVIGATION (Hysteresis)
        #Calculate angle error to target
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
        
        #DEBUG: Log navigazione periodico (ogni 20 tick = 2 secondi)
        if self._debug_tick % 20 == 0:
            angle_deg = math.degrees(angle_diff)
            theta_deg = math.degrees(robot_theta)
            print(f"[DEBUG][MoveToTarget] NAV → {target['name'].upper()} Pos Target {target['x']:.2f}, {target['y']:.2f}| Dist: {distance_to_target:.2f}m | Errore ang: {angle_deg:.1f}° | Azione: {action}")
            print(f"[DEBUG][MoveToTarget] Pos Robot: ({robot_x:.2f}, {robot_y:.2f}) θ={theta_deg:.1f}° | Sensori: L={d_left:.2f} C={d_center:.2f} R={d_right:.2f}")
        
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
        #Home position = spawn platform (saved at startup by InitialRetreat)
        home_pos = self.bb.get("home_position") or HOME_POSITION_DEFAULT
        home_x = home_pos['x']
        home_y = home_pos['y']
        home_theta = home_pos['theta']
        
        #Get current robot position from odometry
        robot_pos = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        current_x = robot_pos.get('x', 0.0)
        current_y = robot_pos.get('y', 0.0)
        
        #Calculate Euclidean distance to home
        distance = ((current_x - home_x)**2 + (current_y - home_y)**2)**0.5
        
        #DEBUG: Log periodico navigazione verso casa
        if not hasattr(self, '_logged_going_home'):
            self._logged_going_home = True
            print(f"[DEBUG][GoHome] Ritorno alla piattaforma di spawn ({home_x:.2f}, {home_y:.2f}) theta={math.degrees(home_theta):.1f}°")
        
        #If close to home (< 0.3m), stop
        if distance < 0.3:
            self.bb.set("plan_action", "STOP")
            print(f"[DEBUG][GoHome] Arrivato alla piattaforma! Distanza: {distance:.2f}m")
            return Status.SUCCESS
        
        #Otherwise, set goal to home and continue moving
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
        GoHome()
    ])
    
    #Retry until mission success
    root = decorators.Retry("LoopUntilSuccess", child=main, num_failures=-1)
    
    return root
