#!/usr/bin/env python3
"""
Charlie Robot - Main Entry Point
Integrated Sense-Plan-Act system loop.
"""

import sys
import argparse
from pathlib import Path
import py_trees

#Charlie module imports
from sense import CharlieDetector, FrameReader
from plan import create_rescue_tree
from act.controller import ActionController
from utils import Visualizer


def main():
    parser = argparse.ArgumentParser(description='Charlie Robot - Emergency Rescue System')
    parser.add_argument('--input', type=str, default=0,
                       help='Input source: 0 for webcam, path to video, or image directory')
    parser.add_argument('--model', type=str, default='models/yolov8n.pt',
                       help='Path to YOLO model')
    parser.add_argument('--conf', type=float, default=0.5,
                       help='Confidence threshold')
    parser.add_argument('--visualize', action='store_true',
                       help='Show visualization window')
    parser.add_argument('--fps', type=int, default=10,
                       help='Processing FPS')
    
    args = parser.parse_args()
    
    print("="*60)
    print("  Charlie Robot - Sense-Plan-Act System (FASE 1)")
    print("="*60)
    print(f"Input: {args.input}")
    print(f"Model: {args.model}")
    print(f"Confidence: {args.conf}")
    print()
    
    #Initialize components
    print("Initializing components...")
    
    try:
        #Sense
        detector = CharlieDetector(model_path=args.model, conf_threshold=args.conf)
        
        #Convert input to appropriate type
        input_source = args.input
        if isinstance(input_source, str) and input_source.isdigit():
            input_source = int(input_source)
        
        reader = FrameReader(source=input_source)
        
        #Plan
        tree = create_rescue_tree()
        tree.setup_with_descendants()
        
        #Blackboard setup
        bb = py_trees.blackboard.Client(name="Charlie")
        bb.register_key("battery", access=py_trees.common.Access.WRITE)
        bb.register_key("fire", access=py_trees.common.Access.WRITE)
        bb.register_key("valve_visible", access=py_trees.common.Access.WRITE)
        bb.register_key("person_visible", access=py_trees.common.Access.WRITE)
        bb.register_key("valve_turned", access=py_trees.common.Access.WRITE)
        
        #Initial state
        bb.set("battery", 100.0)
        bb.set("fire", False)
        bb.set("valve_visible", False)
        bb.set("person_visible", False)
        bb.set("valve_turned", False)
        
        #ACT
        controller = ActionController()
        
        #Visualizer (if enabled)
        viz = Visualizer() if args.visualize else None
        
        print(" All components initialized\n")
        
        #Show tree structure
        print("Behavior Tree Structure:")
        print(py_trees.display.unicode_tree(tree, show_status=True))
        print()
        
    except Exception as e:
        print(f" Initialization failed: {e}")
        return 1
    
    #Main loop
    print("Starting main loop...")
    print("Press 'q' to quit\n")
    
    frame_count = 0
    
    try:
        for frame in reader:
            frame_count += 1
            
            #Sense
            detections = detector.detect(frame)
            
            #Update blackboard with detections
            bb.set("fire", detections['fire'])
            bb.set("person_visible", detections['person'])
            bb.set("valve_visible", detections['valve'])
            
            #Optional: Decrease battery over time (simulation)
            current_battery = bb.get("battery")
            bb.set("battery", max(0, current_battery - 0.1))
            
            #Plan
            tree.tick_once()
            tree_status = tree.status
            
            #ACT
            current_action = controller.get_current_action()
            
            #Visualize
            if viz:
                #Draw detections on frame
                annotated = detector.draw_detections(frame, detections)
                
                #Show with status
                viz.show(annotated, detections, tree_status, current_action)
                
                #Check for quit
                if viz.wait_key(1000 // args.fps) == ord('q'):
                    break
            
            #Console output every 10 frames
            if frame_count % 10 == 0:
                print(f"Frame {frame_count}: Fire={detections['fire']}, "
                      f"Person={detections['person']}, Valve={detections['valve']}, "
                      f"Battery={current_battery:.1f}%")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError in main loop: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        #Cleanup
        print("\nCleaning up...")
        reader.release()
        if viz:
            viz.close()
    
    print(f"\nProcessed {frame_count} frames")
    print("Charlie terminated successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())
