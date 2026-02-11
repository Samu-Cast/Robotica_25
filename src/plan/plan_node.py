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

#ROS2 imports with fallback for testing
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32, Bool
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Range
    from geometry_msgs.msg import Pose2D
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object

#Behavior Tree imports
import py_trees

#Import behaviors from separate module
from behaviors import build_tree

# ============================================================================
# BT VISUALIZATION CONFIG
# ============================================================================
BT_DISPLAY = True           # Set to False to disable BT visualization
BT_DISPLAY_INTERVAL = 50    # Print tree every N ticks (50 = every 5 seconds at 10Hz)
BT_DISPLAY_ON_CHANGE = True # Also print when state changes significantly


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
    'CHARGE_COLOR': 'Front',
    'ACTIVATE_VALVE': 'Stop',
    'MOVE_TO_GOAL': 'Front',
}


class PlanNode(Node):
    """
        Plan ROS2 Node - Strategic decision layer for Charlie Robot.
        
        Subscribes to:
            /sense/odometry - odometry data
            /sense/proximity/front - front distance sensor
            /sense/proximity/front_left - front left distance sensor
            /sense/proximity/front_right - front right distance sensor
            /sense/detection - detection data
            /sense/detection_zone - detection zone data
            /sense/battery - battery data
        
        Publishes to:
            /plan/command - String command for Act (Front, Left, Right, Stop)
            /plan/signals - JSON array of signals (PersonFound, ValveActivated)
            /plan/bt_status - Behavior Tree status
    """
    
    def __init__(self):
        super().__init__('plan_node')
        self.get_logger().info('=== Plan Node Starting ===')
        
        #Build Behavior Tree
        self.tree = build_tree()
        self.bb = py_trees.blackboard.Client(name="PlanNode")
        
        #Register all blackboard keys with write access
        blackboard_keys = [
            'battery', 'obstacles', 'detections', 'targets',
            'current_target', 'visited_targets', 'found', 'signals',
            'plan_action', 'goal_pose', 'mission_complete',
            'detected_color', 'color_area', 'odom_correction', 'detection_zone',
            'detection_distance', 'detection_confidence',
            'distance_left', 'distance_center', 'distance_right',
            'robot_position', 'startup_complete', 'home_position', 'human_position'
        ]
        
        # Track if human position already saved (only save first detection)
        self._human_position_saved = False
        for key in blackboard_keys:
            self.bb.register_key(key, access=py_trees.common.Access.WRITE)
        
        self._init_blackboard()
        self.tree.setup_with_descendants()
        
        # BT visualization state
        self._tick_count = 0
        self._last_bt_state = None
        
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
        self.create_subscription(String, '/sense/detection_zone', self._detection_zone_cb, 10)
        self.create_subscription(Float32, '/sense/battery', self._battery_cb, 10)
        
        #Publishers to Act module
        self.cmd_pub = self.create_publisher(String, '/plan/command', 10)
        self.signals_pub = self.create_publisher(String, '/plan/signals', 10)
        self.bt_status_pub = self.create_publisher(String, '/plan/bt_status', 10)  # BT visualization topic
        
        #Startup synchronization: wait for robot_description + 20 seconds
        self._robot_ready = False
        self._startup_timer = None
        
        #Subscribe to robot_description to know when robot is spawned
        self.create_subscription(
            Clock, '/clock',
            self._clock_cb, 10
        )
        
        self.get_logger().info('Waiting for simulation clock...')
    
    def _clock_cb(self, msg):
        """Callback for /clock topic. Starts BT after simulation time > 0."""
        if not self._robot_ready and self._startup_timer is None:
            #Check if simulation has started (time > 0)
            if msg.clock.sec > 0 or msg.clock.nanosec > 0:
                self.get_logger().info(f'Simulation started (time={msg.clock.sec}s)! Waiting 15s for stabilization...')
                self._startup_timer = self.create_timer(15.0, self._start_behavior_tree)
    
    def _start_behavior_tree(self):
        """Called after clock detected + 15 seconds delay."""
        if self._startup_timer:
            self._startup_timer.cancel()
            self._startup_timer = None
        
        self._robot_ready = True
        self.get_logger().info('=== Plan Node READY - Starting Behavior Tree ===')
        
        #Start the main BT tick timer (10 Hz)
        self.create_timer(0.1, self._tick)
    
    def _init_blackboard(self):
        """Initialize blackboard with default values."""
        #Base data
        self.bb.set("battery", 100.0)  # TODO: Sense doesn't publish battery
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
        self.bb.set("detection_distance", 999.0)
        self.bb.set("detection_confidence", 0.0)
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
    
    def _battery_cb(self, msg):
        """
        Callback for battery level (Float32 percentage 0-100).
        """
        self.bb.set("battery", msg.data)
    
    def _detection_zone_cb(self, msg):
        """
        Callback for detection zone (String: 'left', 'center', 'right', 'none').
        Continuously updated by Sense module for color centering.
        
        Args:
            msg: String message with zone
        """
        zone = msg.data if msg.data in ['left', 'center', 'right', 'none'] else None
        self.bb.set("detection_zone", zone)
    
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
            
            #Set estimated_distance for distance-based decisions
            est_dist = det.get('estimated_distance', -1.0)
            self.bb.set("detection_distance", est_dist if est_dist > 0 else 999.0)
            
            #Set detection confidence
            self.bb.set("detection_confidence", det.get('confidence', 0.0))
            
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
                
                # SAVE HUMAN POSITION on first person detection
                # Use proximity distance to filter (only save if close enough)
                prox_dist = det.get('proximity_distance', 999.0)
                if not self._human_position_saved and prox_dist < 2.0:
                    robot_pos = self.bb.get("robot_position") or {'x': 0.0, 'y': 0.0, 'theta': 0.0}
                    odom_corr = self.bb.get("odom_correction") or {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0}
                    human_pos = {
                        'x': robot_pos.get('x', 0.0) + odom_corr.get('dx', 0.0),
                        'y': robot_pos.get('y', 0.0) + odom_corr.get('dy', 0.0),
                        'theta': robot_pos.get('theta', 0.0) + odom_corr.get('dtheta', 0.0)
                    }
                    self.bb.set("human_position", human_pos)
                    self._human_position_saved = True
                    self.get_logger().info(f"HUMAN POSITION SAVED @ ({human_pos['x']:.2f}, {human_pos['y']:.2f})")
                    
                    # Add signal for person detection with position
                    current_signals = self.bb.get("signals") or []
                    person_signal = {
                        "type": "PersonFound",
                        "x": human_pos['x'],
                        "y": human_pos['y'],
                        "theta": human_pos['theta']
                    }
                    current_signals.append(person_signal)
                    self.bb.set("signals", current_signals)
                    
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
        #Don't tick until robot is ready
        if not self._robot_ready:
            return
        
        self.tree.tick_once()
        self._tick_count += 1
        
        # BT Visualization
        if BT_DISPLAY:
            self._display_behavior_tree()
        
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
    
    def _display_behavior_tree(self):
        """
        Display behavior tree state.
        Publishes to /plan/bt_status topic for separate terminal viewing.
        Use: ros2 topic echo /plan/bt_status
        """
        # Gather current state
        found = self.bb.get("found")
        action = self.bb.get("plan_action")
        current_target = self.bb.get("current_target")
        target_name = current_target.get('name') if current_target else None
        visited = self.bb.get("visited_targets") or []
        
        current_state = (found, action, target_name, tuple(visited))
        
        # Determine if we should publish
        interval_print = (self._tick_count % BT_DISPLAY_INTERVAL == 0)
        state_changed = BT_DISPLAY_ON_CHANGE and (current_state != self._last_bt_state)
        
        if not (interval_print or state_changed):
            return
        
        self._last_bt_state = current_state
        
        # Build status message
        lines = []
        lines.append(f"{'='*60}")
        lines.append(f"[BT] Tick #{self._tick_count} | Action: {action}")
        lines.append(f"[BT] Found: {found} | Target: {target_name}")
        lines.append(f"{'='*60}")
        
        # Tree structure with status
        tree_str = py_trees.display.unicode_tree(
            self.tree,
            show_only_visited=True,
            show_status=True
        )
        lines.append(tree_str)
        
        # Blackboard values
        d_l = self.bb.get("distance_left") or 0
        d_c = self.bb.get("distance_center") or 0  
        d_r = self.bb.get("distance_right") or 0
        color = self.bb.get("detected_color")
        robot_pos = self.bb.get("robot_position") or {}
        
        lines.append(f"[BB] Sensors: L={d_l:.2f}m C={d_c:.2f}m R={d_r:.2f}m")
        lines.append(f"[BB] Color: {color} | Visited: {visited}")
        lines.append(f"[BB] Pos: x={robot_pos.get('x', 0):.2f} y={robot_pos.get('y', 0):.2f} θ={robot_pos.get('theta', 0):.2f}")
        lines.append(f"{'='*60}")
        
        # Publish to topic
        msg = String()
        msg.data = "\n".join(lines)
        self.bt_status_pub.publish(msg)


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
