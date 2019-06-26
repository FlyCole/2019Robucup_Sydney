import time
import dlib
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import correct_orientation


class face_track:
    def __init__(self, session):
        self.session = session
        self.count = 0

        # Initiate Service
        self.Motion = self.session.service("ALMotion")
        self.Memo = self.session.service("ALMemory")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.RobotPos = self.session.service("ALRobotPosture")

        # Initiate Parameters
        self.switch = True
        self.angle = -0.4
        self.if_stop = False
        self.detector = dlib.get_frontal_face_detector()
        self.RobotPos.goToPosture("Stand", .5)
        self.get_image_switch = True
        self.width = 640
        self.height = 480
        self.center_crosswise = 0
        self.center_lengthwise = 0
        self.head_angle = 0
        self.face_ratio = 0
        # Initial Ros publisher and subscriber
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def find_person(self):
        AL_kQVGA = 2
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)
        # create image
        image = np.zeros((self.height, self.width, 3), np.uint8)
        while self.get_image_switch:
            result = self.VideoDev.getImageRemote(nameId)
            if result == None:
                print 'cannot capture.'
            elif result[6] == None:
                print 'no image data string.'
            else:
                values = map(ord, list(str(bytearray(result[6]))))
                i = 0
                for y in range(0, self.height):
                    for x in range(0, self.width):
                        image.itemset((y, x, 0), values[i + 0])
                        image.itemset((y, x, 1), values[i + 1])
                        image.itemset((y, x, 2), values[i + 2])
                        i += 3
                # print image
                cv2.imshow("pepper-top-camera-640*480px", image)
                cv2.waitKey(1)
                self.track(image)
        return "success"

    def track(self, frame):
        frame_copy = frame.copy()
        rects = self.detector(frame_copy, 2)
        k = 0.001
        if len(rects) != 0:
            image_max = 0
            for rect in rects:
                cv2.rectangle(frame_copy, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (0, 0, 255), 2, 8)
                cv2.imshow("track", frame_copy)
                cv2.imwrite("./track.jpg", frame_copy)
                if (rect.right() - rect.left()) * (rect.bottom() - rect.top()) > image_max:
                    image_max = (rect.right() - rect.left()) * (rect.bottom() - rect.top())
                    self.center_lengthwise = (rect.bottom() + rect.top()) / 2
                    self.center_crosswise = (rect.left() + rect.right()) / 2
            self.face_ratio = float(image_max) / float(self.height * self.width)
            if self.face_ratio < .005:
                self.set_velocity(0.15, 0, 0)
                # Correct orientation
                if abs(self.center_crosswise - self.width / 2) > self.width / 10:
                    self.set_velocity(0, 0, 0.001 * (self.width / 2 - self.center_crosswise))
                    time.sleep(2)
                    self.set_velocity(0.15, 0, 0)
                # Correct head position
                if self.center_lengthwise < self.height / 2:
                    self.head_angle -= .1
                    self.Motion.setAngles("Head", [0, self.head_angle], .05)
                elif self.center_lengthwise > (2 * self.height / 3):
                    self.head_angle += .1
                    self.Motion.setAngles("Head", [0, self.head_angle], .05)
                cv2.waitKey(1)
            else:
                if self.face_ratio > .03:
                    self.set_velocity(0, 0, 0)
                    self.get_image_switch = False
                    self.if_stop = True
                else:
                    self.set_velocity(0.15, 0, 0)
                    # Correct orientation
                    if abs(self.center_crosswise - self.width / 2) > self.width / 10:
                        self.set_velocity(0, 0, 0.001 * (self.width / 2 - self.center_crosswise))
                        time.sleep(2)
                        self.set_velocity(0.15, 0, 0)
                        # Correct head position
                    if self.center_lengthwise < self.height / 2:
                        self.head_angle -= .1
                        self.Motion.setAngles("Head", [0, self.head_angle], .05)
                    elif self.center_lengthwise > (2 * self.height / 3):
                        self.head_angle += .1
                        self.Motion.setAngles("Head", [0, self.head_angle], .05)
                cv2.waitKey(1)

    def set_velocity(self, x, y, theta, duration=-1):
        tt = Twist()
        tt.linear.x = x
        tt.linear.y = y
        tt.angular.z = theta
        self.cmd_vel_pub.publish(tt)
        if duration < 0:
            return None
        tic = time.time()
        while time.time() - tic < duration:
            self.cmd_vel_pub.publish(tt)
            time.sleep(0.1)
        tt = Twist()
        tt.linear.x = 0
        tt.linear.y = 0
        tt.angular.z = 0
        self.cmd_vel_pub.publish(tt)