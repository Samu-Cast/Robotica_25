"""
    Demo script for Charlie Robot
    Quick test of the complete system
"""

import sys
from pathlib import Path

#Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from charlie.main import main

if __name__ == "__main__":
    #Run with default webcam
    sys.argv = [
        'demo.py',
        '--input', '0',
        '--visualize',
        '--fps', '10'
    ]
    
    print("Running Charlie demo with webcam...")
    print("If no webcam is available, use: --input path/to/video.mp4")
    print()
    
    sys.exit(main())
