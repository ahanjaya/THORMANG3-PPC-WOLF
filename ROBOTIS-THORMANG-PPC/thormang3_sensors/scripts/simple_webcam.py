#!/usr/bin/env python

import cv2
import time
import rospy

class Webcam:
    def __init__(self):
        rospy.init_node("webcam")
        rospy.loginfo("[Cam] Simple Webcam Running")

        self.cap = cv2.VideoCapture("/dev/WebCam")
        self.cap.set(3, 320) # 800
        self.cap.set(4, 240)  # 600
  
    def run(self):
        while not rospy.is_shutdown():
            start_time = time.time()

            _, frame = self.cap.read()
            cv2.imshow('Tripod', frame)

            ch = 0xFF & cv2.waitKey(1)
            if ch == 27 or ch == ord('q'):
                break

            print("FPS: ", 1.0 / (time.time() - start_time)) # FPS = 1 / time to process loop

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    webcam = Webcam()
    webcam.run()