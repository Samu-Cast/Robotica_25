"""
Charlie Robot - Plan Module
State Machine-based decision making for ROS2

States:
    INIT - Wait for robot_description + 5s
    RETREAT - Move backward from spawn platform
    NAVIGATE - Navigate to current target, avoid obstacles
    AT_TARGET - Arrived at target, check color
    GO_HOME - Return to spawn (valve found)
    DONE - Mission complete

Subscribes to:
    /sense/proximity/front, front_left, front_right (Range)
    /sense/odometry (Pose2D)
    /sense/detection (String JSON)
    /sense/battery (Float32)
    /robot_description (String)

Publishes to:
    /plan/command - String command for Act (Front, Back, Left, Right, Stop)
    /plan/signals - JSON array of signals
"""

import json
import math
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose2D


# Known target locations
KNOWN_TARGETS = {
    'green': {'x': -3.35, 'y': -5.0, 'theta': -1.65},
    'blue': {'x': 0.35, 'y': -4.0, 'theta': 0.0},
    'red': {'x': -6.25, 'y': -1.35, 'theta': 3.0},  # VALVE
}


class PlanNode(Node):
    """Simple State Machine for Charlie Robot navigation."""
    
    # States
    STATE_INIT = 'INIT'
    STATE_RETREAT = 'RETREAT'
    STATE_NAVIGATE = 'NAVIGATE'
    STATE_AT_TARGET = 'AT_TARGET'
    STATE_GO_HOME = 'GO_HOME'
    STATE_DONE = 'DONE'
    
    def __init__(self):
        super().__init__('plan_node')
        self.get_logger().info('=== Plan Node Starting ===')
        
        # State
        self.state = self.STATE_INIT
        self.robot_description_received = False
        self.startup_time = None
        
        # Navigation
        self.home_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.current_target = None
        self.visited_targets = []
        self.retreat_start_time = None
        
        # Sensor data
        self.robot_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.distance_front = 999.0
        self.distance_left = 999.0
        self.distance_right = 999.0
        self.detected_color = None
        self.found = None  # 'person', 'obstacle', or None
        self.detection_zone = None  # 'left', 'center', 'right'
        
        # Subscribers
        self.create_subscription(String, '/robot_description', self._robot_desc_cb, 10)
        self.create_subscription(Range, '/sense/proximity/front', 
                                  lambda m: self._proximity_cb(m, 'front'), 10)
        self.create_subscription(Range, '/sense/proximity/front_left',
                                  lambda m: self._proximity_cb(m, 'left'), 10)
        self.create_subscription(Range, '/sense/proximity/front_right',
                                  lambda m: self._proximity_cb(m, 'right'), 10)
        self.create_subscription(Pose2D, '/sense/odometry', self._odom_cb, 10)
        self.create_subscription(String, '/sense/detection', self._detection_cb, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(String, '/plan/command', 10)
        self.signals_pub = self.create_publisher(String, '/plan/signals', 10)
        
        # Main loop timer (10 Hz)
        self.create_timer(0.1, self._tick)
        
        self.get_logger().info('Waiting for robot_description...')
    
    # ========== CALLBACKS ==========
    
    def _robot_desc_cb(self, msg):
        if not self.robot_description_received:
            self.robot_description_received = True
            self.startup_time = time.time()
            self.get_logger().info('Robot description received! Waiting 5s...')
    
    def _proximity_cb(self, msg, sensor):
        dist = msg.range if msg.range >= 0 else 999.0
        if sensor == 'front':
            self.distance_front = dist
        elif sensor == 'left':
            self.distance_left = dist
        elif sensor == 'right':
            self.distance_right = dist
    
    def _odom_cb(self, msg):
        self.robot_position = {'x': msg.x, 'y': msg.y, 'theta': msg.theta}
    
    def _detection_cb(self, msg):
        try:
            det = json.loads(msg.data)
            self.detected_color = det.get('color')
            self.detection_zone = det.get('zone')
            
            det_type = det.get('type')
            if det_type == 'person':
                self.found = 'person'
            elif det_type == 'obstacle':
                self.found = 'obstacle'
            else:
                self.found = None
        except:
            pass
    
    # ========== HELPERS ==========
    
    def _publish_command(self, cmd):
        """Publish command to Act module."""
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
    
    def _publish_signal(self, signal):
        """Publish signal (PersonFound, ValveActivated)."""
        msg = String()
        msg.data = json.dumps([signal])
        self.signals_pub.publish(msg)
    
    def _distance_to(self, target):
        """Calculate distance to target."""
        dx = target['x'] - self.robot_position['x']
        dy = target['y'] - self.robot_position['y']
        return math.sqrt(dx**2 + dy**2)
    
    def _angle_to(self, target):
        """Calculate angle difference to target (-π to π)."""
        dx = target['x'] - self.robot_position['x']
        dy = target['y'] - self.robot_position['y']
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.robot_position['theta']
        
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff
    
    def _select_next_target(self):
        """Select closest unvisited target."""
        closest = None
        min_dist = float('inf')
        
        for name, pos in KNOWN_TARGETS.items():
            if name in self.visited_targets:
                continue
            dist = self._distance_to(pos)
            if dist < min_dist:
                min_dist = dist
                closest = {'name': name, **pos}
        
        return closest
    
    def _check_obstacle(self):
        """Check if obstacle detected by sensors."""
        return self.distance_front < 0.5 or self.distance_left < 0.3 or self.distance_right < 0.3
    
    def _avoid_obstacle(self):
        """Return avoidance command."""
        # If person/obstacle from camera, use detection zone
        if self.found in ['person', 'obstacle']:
            if self.found == 'person':
                self._publish_signal('PersonFound')
            
            if self.detection_zone == 'left':
                return 'Right'
            elif self.detection_zone == 'right':
                return 'Left'
        
        # Use ultrasonic sensors
        if self.distance_left > self.distance_right:
            return 'Left'
        else:
            return 'Right'
    
    def _navigate_to(self, target):
        """Return navigation command to reach target."""
        angle_diff = self._angle_to(target)
        
        # Thresholds
        ANGLE_OK = 0.2  # ~11 degrees
        
        if abs(angle_diff) < ANGLE_OK:
            return 'Front'
        elif angle_diff > 0:
            return 'FrontLeft'
        else:
            return 'FrontRight'
    
    # ========== STATE MACHINE ==========
    
    def _tick(self):
        """Main state machine loop at 10Hz."""
        
        # === STATE: INIT ===
        if self.state == self.STATE_INIT:
            self._publish_command('Stop')
            
            if self.robot_description_received:
                elapsed = time.time() - self.startup_time
                if elapsed >= 5.0:
                    # Save spawn as home
                    self.home_position = self.robot_position.copy()
                    self.get_logger().info(f'Home saved: ({self.home_position["x"]:.2f}, {self.home_position["y"]:.2f})')
                    
                    self.retreat_start_time = time.time()
                    self.state = self.STATE_RETREAT
                    self.get_logger().info('STATE: RETREAT')
            return
        
        # === STATE: RETREAT ===
        if self.state == self.STATE_RETREAT:
            elapsed = time.time() - self.retreat_start_time
            
            if elapsed < 4.0:
                self._publish_command('Back')
            else:
                self._publish_command('Stop')
                
                # Select first target
                self.current_target = self._select_next_target()
                if self.current_target:
                    self.get_logger().info(f'Target selected: {self.current_target["name"].upper()}')
                    self.state = self.STATE_NAVIGATE
                    self.get_logger().info('STATE: NAVIGATE')
                else:
                    self.get_logger().error('No targets available!')
            return
        
        # === STATE: NAVIGATE ===
        if self.state == self.STATE_NAVIGATE:
            if not self.current_target:
                self.current_target = self._select_next_target()
                if not self.current_target:
                    self.get_logger().info('All targets visited, no valve found!')
                    self.state = self.STATE_DONE
                    return
            
            # Check if arrived at target
            dist = self._distance_to(self.current_target)
            if dist < 1.0 and self.distance_front < 0.8:
                self._publish_command('Stop')
                self.state = self.STATE_AT_TARGET
                self.get_logger().info(f'STATE: AT_TARGET ({self.current_target["name"].upper()})')
                return
            
            # Check for obstacles/persons
            if self._check_obstacle() or self.found in ['person', 'obstacle']:
                cmd = self._avoid_obstacle()
                self._publish_command(cmd)
                return
            
            # Navigate toward target
            cmd = self._navigate_to(self.current_target)
            self._publish_command(cmd)
            return
        
        # === STATE: AT_TARGET ===
        if self.state == self.STATE_AT_TARGET:
            self._publish_command('Stop')
            
            target_name = self.current_target['name']
            
            # Mark as visited
            if target_name not in self.visited_targets:
                self.visited_targets.append(target_name)
                self.get_logger().info(f'Target {target_name.upper()} visited')
            
            # Check if valve (red)
            if self.detected_color == 'red' or target_name == 'red':
                self.get_logger().info('=== VALVE FOUND! ===')
                self._publish_signal('ValveActivated')
                self.state = self.STATE_GO_HOME
                self.get_logger().info('STATE: GO_HOME')
                return
            
            # Not valve, select next target
            self.current_target = self._select_next_target()
            if self.current_target:
                self.get_logger().info(f'Next target: {self.current_target["name"].upper()}')
                self.state = self.STATE_NAVIGATE
                self.get_logger().info('STATE: NAVIGATE')
            else:
                self.get_logger().info('All targets visited!')
                self.state = self.STATE_DONE
            return
        
        # === STATE: GO_HOME ===
        if self.state == self.STATE_GO_HOME:
            dist = self._distance_to(self.home_position)
            
            if dist < 0.5:
                self._publish_command('Stop')
                self.get_logger().info('=== MISSION COMPLETE ===')
                self.state = self.STATE_DONE
                return
            
            # Check for obstacles
            if self._check_obstacle() or self.found in ['person', 'obstacle']:
                cmd = self._avoid_obstacle()
                self._publish_command(cmd)
                return
            
            # Navigate home
            cmd = self._navigate_to(self.home_position)
            self._publish_command(cmd)
            return
        
        # === STATE: DONE ===
        if self.state == self.STATE_DONE:
            self._publish_command('Stop')
            return


def main(args=None):
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
