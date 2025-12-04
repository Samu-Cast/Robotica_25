"""
    YOLOv8 Detector per Charlie
    Wrapper per ultralytics YOLOv8
"""

from ultralytics import YOLO
import cv2
import numpy as np


class CharlieDetector:
    """
        YOLOv8 detector per fire, person, valve
    """
    
    def __init__(self, model_path='models/yolov8n.pt', conf_threshold=0.5):
        """
            Args:
                model_path: Path to YOLO model weights
                conf_threshold: Confidence threshold for detections
        """
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        
        #Custom classes (se usiamo modello custom trained)
        self.custom_classes = {
            0: 'fire',
            1: 'person',
            2: 'valve'
        }
    
    def detect(self, frame):
        """
            Detect objects in frame using YOLO
        
            Args:
                frame: numpy array (BGR image)
            
            Returns:
            dict: {
                'fire': bool,
                'person': bool,
                'valve': bool,
                'detections': [
                    {
                        'class': str,
                        'confidence': float,
                        'bbox': [x1, y1, x2, y2]
                    },
                    ...
                ]
            }
        """
        if frame is None:
            return self._empty_result()
        
        #YOLO inference
        results = self.model(frame, conf=self.conf_threshold, verbose=False)
        
        #Process results
        detections = []
        fire_detected = False
        person_detected = False
        valve_detected = False
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                #Get box data
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                
                #Get class name
                class_name = result.names[cls] if hasattr(result, 'names') else str(cls)
                
                #Check for our target classes
                if 'fire' in class_name.lower():
                    fire_detected = True
                    class_name = 'fire'
                elif 'person' in class_name.lower():
                    person_detected = True
                    class_name = 'person'
                elif 'valve' in class_name.lower():
                    valve_detected = True
                    class_name = 'valve'
                
                detections.append({
                    'class': class_name,
                    'confidence': conf,
                    'bbox': [float(x1), float(y1), float(x2), float(y2)]
                })
        
        return {
            'fire': fire_detected,
            'person': person_detected,
            'valve': valve_detected,
            'detections': detections
        }
    
    def _empty_result(self):
        """Return empty detection result"""
        return {
            'fire': False,
            'person': False,
            'valve': False,
            'detections': []
        }
    
    def draw_detections(self, frame, detections):
        """
        Draw bounding boxes on frame
        
        Args:
            frame: Original frame
            detections: Detection results from detect()
            
        Returns:
            frame with boxes drawn
        """
        annotated = frame.copy()
        
        for det in detections['detections']:
            x1, y1, x2, y2 = [int(v) for v in det['bbox']]
            class_name = det['class']
            conf = det['confidence']
            
            # Color based on class
            color = self._get_color(class_name)
            
            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{class_name}: {conf:.2f}"
            cv2.putText(annotated, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return annotated
    
    def _get_color(self, class_name):
        """Get color for class"""
        colors = {
            'fire': (0, 0, 255),      #Red
            'person': (255, 0, 0),     #Blue
            'valve': (0, 255, 0)       #Green
        }
        return colors.get(class_name, (255, 255, 255))


if __name__ == "__main__":
    # Test
    print("Initializing YOLO detector...")
    detector = CharlieDetector()
    
    # Test with dummy image
    test_image = np.zeros((480, 640, 3), dtype=np.uint8)
    results = detector.detect(test_image)
    
    print(f"Results: {results}")
    print("✓ Detector initialized successfully")
