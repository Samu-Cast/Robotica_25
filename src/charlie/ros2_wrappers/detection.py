"""
    Detection algorithms per Charlie
    Fire detection, person detection, valve detection
"""

import cv2
import numpy as np
from logger import Logger


class FireDetector:
    """Fire detector using OpenCV"""
    
    def __init__(self):
        self.logger = Logger("detection_log.md", clear=True)
        self.logger.section("Fire Detector Initialized")
    
    def detect_fire(self, image):
        """
        Detect fire in an RGB image
        Uses color thresholding (red/orange)
        """
        if image is None:
            return False, None
        
        #Convert in HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        #Range per colori fuoco (rosso/arancione/giallo)
        lower_fire = np.array([0, 100, 100])
        upper_fire = np.array([35, 255, 255])
        
        #Mask
        mask = cv2.inRange(hsv, lower_fire, upper_fire)
        
        #Count pixel
        fire_pixels = cv2.countNonZero(mask)
        threshold = 100  #Min pixel for fire
        
        detected =fire_pixels > threshold
        
        if detected:
            self.logger.warning(f"Fire detected! Pixels: {fire_pixels}")
        
        return detected, mask


class PersonDetector:
    """Person detector using OpenCV"""
    
    def __init__(self):
        self.logger = Logger("detection_log.md")
        # TODO: Load ML model (YOLO, Haar Cascade, etc.)
    
    def detect_person(self, image):
        """Detect person in an image"""
        if image is None:
            return False, []
        
        # TODO: Implement detection
        # Options:
        # - Haar Cascade per face detection
        # - HOG descriptor
        # - YOLO/SSD per person detection
        
        detected = False
        bboxes = []
        
        return detected, bboxes


class ValveDetector:
    """Valve detector using OpenCV"""
    
    def __init__(self):
        self.logger = Logger("detection_log.md")
    
    def detect_valve(self, image):
        """Detect valve in an image"""
        if image is None:
            return False, None
        
        # TODO: Implement detection
        # Options:
        # - Template matching
        # - Shape detection (cerchi, rettangoli)
        # - Feature matching (SIFT/ORB)
        
        detected = False
        position = None
        
        return detected, position


# Test
if __name__ == "__main__":
    detector = FireDetector()
    
    # Test with dummy image
    test_image = np.zeros((480, 640, 3), dtype=np.uint8)
    # Add "fire" zone (red)
    test_image[200:300, 300:400] = [0, 0, 255]  # BGR
    
    detected, mask = detector.detect_fire(test_image)
    print(f"Fire detected: {detected}")
