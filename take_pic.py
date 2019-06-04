# -*- encoding: UTF-8 -*-
"""
    Author: Yifei Ren
    Name: take_pic
    Version: 1.0
    Date: 03/06/2019
    Description: Take picture using frontal_face_detector.
                 Press "s" when you want to save picture.
                 Press "Esc" when you want to exit.
    Note: When the file is used in pepper, make sure the camera can be detected.
"""

import cv2
import rospy
from std_msgs.msg import String


def capture_pic(msg):
    cap = cv2.VideoCapture(0)
    i = 0
    ret, frame = cap.read()
    cv2.imwrite('capture.jpg', frame)
    cv2.imshow("capture", frame)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node("publish")
    signal_sub = rospy.Subscriber("/patient_reach", String, capture_pic)
    rospy.spin()