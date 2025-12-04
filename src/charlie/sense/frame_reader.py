"""
    Frame Reader for Charlie
    Read frames from video file, webcam, or image directory
"""

import cv2
import os
from pathlib import Path


class FrameReader:
    """
        Unified frame reader for different input sources
    """
    
    def __init__(self, source=0):
        """
            Args:
                source: int (webcam), str (video path), or Path (image directory)
        """
        self.source = source
        self.cap = None
        self.is_video = False
        self.is_webcam = False
        self.is_images = False
        self.image_paths = []
        self.current_idx = 0
        
        self._initialize()
    
    def _initialize(self):
        """Initialize the appropriate reader"""
        if isinstance(self.source, int):
            #Webcam
            self.cap = cv2.VideoCapture(self.source)
            self.is_webcam = True
            
        elif isinstance(self.source, (str, Path)):
            path = Path(self.source)
            
            if path.is_file():
                #Video file
                self.cap = cv2.VideoCapture(str(path))
                self.is_video = True
                
            elif path.is_dir():
                #Image directory
                self.image_paths = sorted([
                    p for p in path.glob('*')
                    if p.suffix.lower() in ['.jpg', '.jpeg', '.png', '.bmp']
                ])
                self.is_images = True
                
            else:
                raise ValueError(f"Invalid source: {self.source}")
        else:
            raise ValueError("Source must be int, str, or Path")
    
    def read(self):
        """
            Read next frame
        
            Returns:
                tuple: (success, frame) or (False, None) if no more frames
        """
        if self.is_video or self.is_webcam:
            return self.cap.read()
            
        elif self.is_images:
            if self.current_idx >= len(self.image_paths):
                return False, None
            
            img_path = self.image_paths[self.current_idx]
            frame = cv2.imread(str(img_path))
            self.current_idx += 1
            
            return frame is not None, frame
        
        return False, None
    
    def __iter__(self):
        """Make iterator"""
        return self
    
    def __next__(self):
        """Get next frame"""
        success, frame = self.read()
        if not success:
            raise StopIteration
        return frame
    
    def release(self):
        """Release resources"""
        if self.cap is not None:
            self.cap.release()
    
    def get_fps(self):
        """Get FPS (for video sources)"""
        if self.cap is not None:
            return self.cap.get(cv2.CAP_PROP_FPS)
        return 30  # Default
    
    def get_total_frames(self):
        """Get total frame count"""
        if self.is_video:
            return int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        elif self.is_images:
            return len(self.image_paths)
        return -1  # Unknown for webcam
    
    def __del__(self):
        """Cleanup"""
        self.release()


if __name__ == "__main__":
    # Test with webcam
    print("Testing FrameReader with webcam...")
    reader = FrameReader(0)
    
    success, frame = reader.read()
    if success:
        print(f"✓ Read frame: {frame.shape}")
    else:
        print("✗ No webcam available (normal if no camera)")
    
    reader.release()
