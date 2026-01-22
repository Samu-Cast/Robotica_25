"""
Human Detector - Uses YOLOv8 Nano (lightest model)
Downloads automatically on first use (~6MB)
Detects "person" class from COCO dataset
"""

import os
from pathlib import Path

try:
    import torch
except Exception:
    torch = None

class HumanDetector:
    """
    YOLO-based human detector using YOLOv8 Nano (lightweight)
    """
    
    # COCO class ID for person
    PERSON_CLASS_ID = 0
    
    def __init__(self, model_name='yolov8n.pt', conf_threshold=0.5):
        """
        Args:
            model_name: YOLO model name (downloads automatically if not present)
            conf_threshold: Confidence threshold for detections
        """
        self.conf_threshold = conf_threshold
        self.model = None
        

        if torch is not None:
            try:
                torch.backends.nnpack.enabled = False
            except Exception:
                pass
        
        try:
            from ultralytics import YOLO
        except ImportError:
            print("Warning: ultralytics not installed. Human detection disabled.")
            return
        
        try:
            # YOLOv8 Nano - smallest and fastest, ~6MB
            # Downloads automatically on first use
            self.model = YOLO(model_name)
            print(f"Human detector loaded: {model_name}")
        except Exception as e:
            print(f"Error loading YOLO model: {e}")
            self.model = None
    
    def detect(self, frame):
        """
        Detect humans in frame
        
        Args:
            frame: numpy array (BGR image from OpenCV)
            
        Returns:
            dict: {
                'person_detected': bool,
                'persons': [
                    {
                        'confidence': float,
                        'bbox': [x1, y1, x2, y2],
                        'center': (cx, cy)
                    },
                    ...
                ]
            }
        """
        if self.model is None or frame is None:
            return self._empty_result()
        
        try:
            # Run YOLO inference - filter for person class only
            results = self.model(
                frame, 
                conf=self.conf_threshold, 
                classes=[self.PERSON_CLASS_ID],  # Only detect persons
                imgsz=320,  # Reduced resolution for faster inference
                verbose=False
            )
            
            persons = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get box data
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    
                    # Calculate center
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    
                    persons.append({
                        'confidence': conf,
                        'bbox': [float(x1), float(y1), float(x2), float(y2)],
                        'center': (float(cx), float(cy))
                    })
            
            return {
                'person_detected': len(persons) > 0,
                'persons': persons
            }
            
        except Exception as e:
            print(f"Human detection error: {e}")
            return self._empty_result()
    
    def _empty_result(self):
        """Return empty detection result"""
        return {
            'person_detected': False,
            'persons': []
        }


# Singleton instance for easy import
_detector_instance = None

def get_human_detector():
    """Get singleton instance of HumanDetector"""
    global _detector_instance
    if _detector_instance is None:
        _detector_instance = HumanDetector()
    return _detector_instance


if __name__ == "__main__":
    import cv2
    import numpy as np
    
    print("Testing Human Detector with YOLOv8 Nano...")
    detector = HumanDetector()
    
    # Test with dummy image
    test_image = np.zeros((480, 640, 3), dtype=np.uint8)
    results = detector.detect(test_image)
    
    print(f"Results: {results}")
    print("✓ Human detector test complete")
