"""
    Visualizer for Charlie
    Display detections and behavior tree status
"""

import cv2
import numpy as np


class Visualizer:
    """Display detections and system status"""
    
    def __init__(self, window_name="Charlie Robot"):
        self.window_name = window_name
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
    
    def show(self, frame, detections=None, tree_status=None, controller_action=None):
        """
        Display frame with overlays
        
        Args:
            frame: Original frame
            detections: Detection results
            tree_status: Behavior tree status
            controller_action: Current action from controller
        """
        display = frame.copy()
        
        # Add detection overlay
        if detections:
            display = self._draw_detection_overlay(display, detections)
        
        # Add status panel
        display = self._draw_status_panel(display, detections, tree_status, controller_action)
        
        cv2.imshow(self.window_name, display)
    
    def _draw_detection_overlay(self, frame, detections):
        """Draw detection indicators"""
        h, w = frame.shape[:2]
        y_offset = 30
        
        #Fire indicator
        if detections.get('fire'):
            cv2.putText(frame, "FIRE DETECTED!", (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            y_offset += 40
        
        #Person indicator
        if detections.get('person'):
            cv2.putText(frame, "PERSON DETECTED", (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            y_offset += 35
        
        #Valve indicator
        if detections.get('valve'):
            cv2.putText(frame, "VALVE DETECTED", (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        return frame
    
    def _draw_status_panel(self, frame, detections, tree_status, action):
        """Draw status panel at bottom"""
        h, w = frame.shape[:2]
        
        # Create semi-transparent panel
        overlay = frame.copy()
        panel_height = 100
        cv2.rectangle(overlay, (0, h - panel_height), (w, h), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.7, frame, 0.3, 0)
        
        # Status text
        y_offset = h - 75
        
        # Detection counts
        if detections:
            det_count = len(detections.get('detections', []))
            cv2.putText(frame, f"Detections: {det_count}", (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Tree status
        if tree_status is not None:
            cv2.putText(frame, f"Tree: {tree_status}", (10, y_offset + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Current action
        if action:
            cv2.putText(frame, f"Action: {action}", (10, y_offset + 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return frame
    
    def wait_key(self, delay=1):
        """Wait for key press"""
        return cv2.waitKey(delay) & 0xFF
    
    def close(self):
        """Close windows"""
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # Test
    viz = Visualizer()
    
    # Create test frame
    test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    test_detections = {
        'fire': True,
        'person': False,
        'valve': True,
        'detections': [{'class': 'fire'}]
    }
    
    viz.show(test_frame, test_detections, "SUCCESS", "MOVE_FORWARD")
    
    print("Press any key to close...")
    viz.wait_key(0)
    viz.close()
