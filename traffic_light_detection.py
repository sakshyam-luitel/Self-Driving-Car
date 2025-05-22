import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Setup GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering

# Motor pins (adjust to your wiring)
m11 = 16
m12 = 12
m21 = 21
m22 = 20

# Setup motor control pins
GPIO.setup(m11, GPIO.OUT)
GPIO.setup(m12, GPIO.OUT)
GPIO.setup(m21, GPIO.OUT)
GPIO.setup(m22, GPIO.OUT)

# Functions to control motor movement
def stop():
    print('Stop')
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)

def forward():
    print('Forward')
    GPIO.output(m11, 0)
    GPIO.output(m12, 1)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)

# Capturing video from file
cap = cv2.VideoCapture('vids/Lane_vid.avi')

# Start moving forward initially
forward()

while cap.isOpened():
    _, img1 = cap.read()
    if img1 is None:  # In case the video file ends
        break
    
    # Define region of interest (ROI)
    img = img1[30:2000, 500:700]

    # Convert to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define color ranges
    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    green_lower = np.array([66, 122, 129], np.uint8)
    green_upper = np.array([86, 255, 255], np.uint8)

    # Mask for red and green colors
    red = cv2.inRange(hsv, red_lower, red_upper)
    green = cv2.inRange(hsv, green_lower, green_upper)

    # Morphological transformation - Dilation
    kernel = np.ones((5, 5), "uint8")
    red = cv2.dilate(red, kernel)
    res = cv2.bitwise_and(img, img, mask=red)

    green = cv2.dilate(green, kernel)
    res2 = cv2.bitwise_and(img, img, mask=green)

    # Track red color (stop sign)
    contours, _ = cv2.findContours(red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 300:  # Adjust area threshold based on your needs
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img, "RED color", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
            print('Red detected')
            stop()  # Stop if red is detected

    # Track green color (go signal)
    contours, _ = cv2.findContours(green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 300:  # Adjust area threshold based on your needs
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, "Green color", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))
            print('Green detected')
            forward()  # Move forward if green is detected

    # Show the processed frames (for debugging purposes)
    cv2.imshow("Color Tracking", img)

    # Break on pressing 'q'
    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

# Cleanup
GPIO.cleanup()
cap.release()
cv2.destroyAllWindows()
