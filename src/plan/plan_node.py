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
            /sense/world_state - JSON with battery, odometry, obstacles, detections, detected_color
        
        Publishes to:
            /plan/command - String command for Act (Front, Left, Right, Stop)
            /plan/signals - JSON array of signals (PersonFound, ValveActivated)
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
            'robot_position', 'startup_complete', 'home_position', 'bumper', 'bumper_event'
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
        self.create_subscription(String, '/sense/detection_zone', self._detection_zone_cb, 10)
        self.create_subscription(Float32, '/sense/battery', self._battery_cb, 10)
        self.create_subscription(Bool, '/sense/bumper', self._bumper_cb, 10)
        self.create_subscription(Bool, '/sense/bumper_event', self._bumper_event_cb, 10)
        
        #Publishers to Act module
        self.cmd_pub = self.create_publisher(String, '/plan/command', 10)
        self.signals_pub = self.create_publisher(String, '/plan/signals', 10)
        
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
                self.get_logger().info(f'Simulation started (time={msg.clock.sec}s)! Waiting 20s for stabilization...')
                self._startup_timer = self.create_timer(60.0, self._start_behavior_tree)
    
    def _start_behavior_tree(self):
        """Called after clock detected + 20 seconds delay."""
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
        self.bb.set("bumper", False)
        self.bb.set("bumper_event", False)
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
    
    def _bumper_cb(self, msg):
        """
        Callback for bumper sensor (Bool) - continuous state.
        """
        previous_bumper = self.bb.get("bumper")
        self.bb.set("bumper", msg.data)
        
        if msg.data and not previous_bumper:
            self.get_logger().info("🔴 [PLAN] Bumper state changed to TRUE")
    
    def _detection_zone_cb(self, msg):
        """
        Callback for detection zone (String: 'left', 'center', 'right', 'none').
        Continuously updated by Sense module for color centering.
        
        Args:
            msg: String message with zone
        """
        zone = msg.data if msg.data in ['left', 'center', 'right', 'none'] else None
        self.bb.set("detection_zone", zone)
    
    def _bumper_event_cb(self, msg):
        """
        Callback for bumper event (Bool - True only when collision is detected).
        This is the edge-triggered version of the bumper sensor.
        
        Args:
            msg: Bool message (True = collision just happened)
        """
        self.bb.set("bumper_event", False)
        if msg.data:  # Only set bumper when collision is detected
            self.bb.set("bumper_event", True)
            self.get_logger().warn("🔴 [BUMPER_EVENT] COLLISION EVENT RECEIVED - Bumper set to TRUE")
        else:
            # When bumper_event goes False, we don't reset bumper here
            # AtTarget is responsible for resetting bumper after processing
            pass
    
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
