import math
import cv2
import numpy as np
from ultralytics import YOLO

# Function to calculate slope
def slope(vx1, vx2, vy1, vy2):
    if vx2 - vx1 == 0:
        return 90
    m = float(vy2 - vy1) / float(vx2 - vx1)
    theta1 = math.atan(m)
    return theta1 * (180 / np.pi)

# Initialize video capture
cap = cv2.VideoCapture('vids/signs_forward.mp4')

# Initialize traffic sign detector
detector = YOLO("./model/traffic_sign_detector.pt", task="detect")

while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        break

    img = cv2.resize(img, (600, 600))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # Region of Interest (ROI) Mask
    mask = np.zeros_like(edges)
    height, width = img.shape[:2]
    roi_vertices = np.array([[(50, height), (width - 50, height), (width // 2 + 50, height // 2), (width // 2 - 50, height // 2)]], dtype=np.int32)
    cv2.fillPoly(mask, roi_vertices, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    # Detect lane lines
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 25, minLineLength=40, maxLineGap=40)

    left_lane = []
    right_lane = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = slope(x1, x2, y1, y2)

            if -80 <= angle <= -30:  # Right lane (Negative slope)
                right_lane.append((x1, y1))
                right_lane.append((x2, y2))
            elif 30 <= angle <= 80:  # Left lane (Positive slope)
                left_lane.append((x1, y1))
                left_lane.append((x2, y2))

    # Sort lane points for smooth polygon filling
    left_lane = sorted(left_lane, key=lambda p: p[1])  # Sort by Y coordinate (bottom to top)
    right_lane = sorted(right_lane, key=lambda p: p[1], reverse=True)  # Sort in reverse

    if left_lane and right_lane:
        lane_polygon = np.array(left_lane + right_lane, dtype=np.int32)
        lane_mask = np.zeros_like(img)
        cv2.fillPoly(lane_mask, [lane_polygon], (0, 255, 255))  # Fill lane with yellow

        img = cv2.addWeighted(img, 1, lane_mask, 0.5, 0)  # Overlay lane mask

        # Draw lane boundaries in white
        for i in range(len(left_lane) - 1):
            cv2.line(img, left_lane[i], left_lane[i + 1], (255, 255, 255), 2)
        for i in range(len(right_lane) - 1):
            cv2.line(img, right_lane[i], right_lane[i + 1], (255, 255, 255), 2)

    # Traffic sign detection
    results = detector(img)
    for result in results:
        for bbox in result.boxes:
            x1, y1, x2, y2 = map(int, bbox.xyxy[0])  # Bounding box
            conf = bbox.conf[0]  # Confidence score
            cls = int(bbox.cls[0])  # Class index
            label = detector.names[cls]  # Class name

            # Draw bounding box and label
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            text = f"{label} ({conf:.2f})"
            cv2.putText(img, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Show results
    cv2.imshow("Lane Detection", img)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()