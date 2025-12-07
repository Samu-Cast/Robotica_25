# Charlie Robot - FASE 1 Development

## Setup

```bash
#Install dependencies
pip install -r requirements.txt

#Download YOLOv8 model
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
mv yolov8n.pt models/
```

## Usage

### Run with webcam
```bash
python src/charlie/main.py --input 0 --visualize
```

### Run with video file
```bash
python src/charlie/main.py --input data/test/video.mp4 --visualize
```

### Quick demo
```bash
python scripts/demo.py
```

## Structure

```
src/charlie/
├── sense/          # YOLO detection
├── plan/           # Behavior tree  
├── act/            # Action control
└── utils/          # Visualizer
```

## Next Steps

1. Generate/download dataset (fire, person, valve)
2. Train custom YOLO model (optional)
3. Test with real scenarios
4. FASE 2: Integrate with ROS2 + Gazebo
