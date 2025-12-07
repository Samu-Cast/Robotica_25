"""
    Sense module - Object detection and perception
"""

from .yolo_detector import CharlieDetector
from .frame_reader import FrameReader

__all__ = ['CharlieDetector', 'FrameReader']
