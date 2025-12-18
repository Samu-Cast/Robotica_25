"""
Human Detector - Uses YOLO model trained on humans
Uses the model from YOLO/humanDetection/best.pt
"""

import os
from pathlib import Path

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: ultralytics not installed. Human detection disabled.")


class HumanDetector:
    """
    YOLO-based human detector using custom trained model
    """
    
    def __init__(self, model_path=None, conf_threshold=0.5):
        """
        Args:
            model_path: Path to YOLO model weights (best.pt)
            conf_threshold: Confidence threshold for detections
        """
        self.conf_threshold = conf_threshold
        self.model = None
        
        if not YOLO_AVAILABLE:
            return
        
        # Find model path
        if model_path is None:
            # Default path relative to this file
            current_dir = Path(__file__).parent
            model_path = current_dir / 'YOLO' / 'humanDetection' / 'best.pt'
        
        model_path = Path(model_path)
        
        if model_path.exists():
            try:
                self.model = YOLO(str(model_path))
                print(f"Human detector loaded: {model_path}")
            except Exception as e:
                print(f"Error loading YOLO model: {e}")
                self.model = None
        else:
            print(f"Warning: Model not found at {model_path}")
    
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
            # Run YOLO inference
            results = self.model(frame, conf=self.conf_threshold, verbose=False)
            
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
    
    print("Testing Human Detector...")
    detector = HumanDetector()
    
    # Test with dummy image
    test_image = np.zeros((480, 640, 3), dtype=np.uint8)
    results = detector.detect(test_image)
    
    print(f"Results: {results}")
    print("✓ Human detector test complete")
