"""
    Action Controller for Charlie (FASE 1)
    Executes actions based on behavior tree decisions
"""


class ActionController:
    """
    Controls robot actions
    FASE 1: Only logging
    FASE 2: Will publish to ROS2 topics
    """
    
    def __init__(self):
        self.current_action = None
    
    def move_forward(self, speed=0.2):
        """Move robot forward"""
        self.current_action = f"MOVE_FORWARD({speed})"
        print(f"ACTION: Moving forward at speed {speed}")
        #FASE 2: publish to /cmd_vel
    
    def rotate(self, angular_speed=0.5):
        """Rotate robot"""
        self.current_action = f"ROTATE({angular_speed})"
        print(f"ACTION: Rotating at {angular_speed} rad/s")
        #FASE 2: publish to /cmd_vel
    
    def stop(self):
        """Stop robot"""
        self.current_action = "STOP"
        print("ACTION: Stopping")
        #FASE 2: publish zero velocities
    
    def approach_target(self, target_type):
        """Approach detected target"""
        self.current_action = f"APPROACH_{target_type.upper()}"
        print(f"ACTION: Approaching {target_type}")
        #FASE 2: path planning + movement
    
    def signal(self, message):
        """Send signal/alert"""
        self.current_action = f"SIGNAL"
        print(f"SIGNAL: {message}")
        #FASE 2: publish to alert topic
    
    def explore(self):
        """Execute exploration behavior"""
        self.current_action = "EXPLORE"
        print("ACTION: Exploring environment")
        #FASE 2: autonomous exploration
    
    def get_current_action(self):
        """Get current action being executed"""
        return self.current_action


if __name__ == "__main__":
    # Test
    controller = ActionController()
    
    controller.move_forward(0.3)
    controller.rotate(0.5)
    controller.approach_target('valve')
    controller.signal('Fire detected!')
    controller.stop()
    
    print(f"\nLast action: {controller.get_current_action()}")
