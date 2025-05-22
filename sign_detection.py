# import libraries
from ultralytics import YOLO
import cv2

# initialize the detector model
detector = YOLO("./model/traffic_sign_detector.pt", task="detect")

# video path
video_path = "vids/signs_forward.mp4"

# read the video file
cap = cv2.VideoCapture(video_path)

# check for errors
if not cap.isOpened():
    print("Unable to open the input file")
    exit()

# processing
ret = True

while ret:
    ret, frame = cap.read()
    
    if not ret:
        break

    # Perform detection
    results = detector(frame)
    
    # Get the predictions from the results (results[0] contains detection results)
    boxes = results[0].boxes.xyxy.cpu().numpy()  # Bounding boxes (xyxy format)
    class_ids = results[0].boxes.cls.cpu().numpy()  # Class IDs of the detected objects
    
    # Iterate through all the detections
    for i, box in enumerate(boxes):
        x1, y1, x2, y2 = box  # Get bounding box coordinates
        
        # Convert to integers
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        
        # Draw rectangle around the detected traffic sign
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Get class name using the class ID
        class_name = results[0].names[int(class_ids[i])]
        
        # Display the class name on the image
        cv2.putText(frame, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    
    # Display the video with annotated traffic signs
    cv2.imshow("Traffic sign detector", frame)
    
    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

# Uncomment the following line to detect and save the annotated images without visualizing them
# results = detector(video_path, save=True)
