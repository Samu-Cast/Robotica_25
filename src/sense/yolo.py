# Import the YOLO module from the ultralytics library.
from ultralytics import YOLO

# Create an instance of the YOLO model
# Initialize it with the pre-trained weights file 'best.pt' located at the specified path
model = YOLO('../../models/YOLO/humanDetection/best.pt')

# Define path to video file
source = '../../models/YOLO/humanDetection/inputs'

# Run inference on the source
results = model(source, stream=True)  # generator of Results objects

# Iterate over each 'result' in the 'results' collection
for result in results:
    # Extract bounding box information from the current 'result'
    boxes = result.boxes
    # Extract mask information from the current 'result'
    masks = result.masks
    # Extract keypoints information from the current 'result'
    keypoints = result.keypoints
    # Extract probability scores from the current 'result'
    probs = result.probs
    
# Now you can use 'boxes', 'masks', 'keypoints', and 'probs' as needed for further processing or analysis.
# These variables hold the relevant information related to the object detection results for the current iteration.