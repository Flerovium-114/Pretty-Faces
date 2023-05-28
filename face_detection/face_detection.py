#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Int32MultiArray

def main():

    face_cascade = cv2.CascadeClassifier('/home/flerovium114/catkin_ws/src/face_detection/haarcascade_frontalface_default.xml')

    # Check if the cascade classifier is loaded successfully
    if face_cascade.empty():
        print("Error loading cascade classifier XML file.")
        return

    video_capture = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = video_capture.read()

        # Convert the frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Perform face detection
        faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Draw rectangles around the detected faces
	
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
	    
	
	

	# Publish face locations as ROS message
   	

        # Display the resulting frame
        cv2.imshow('Face Detection', frame)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close the windows
    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


