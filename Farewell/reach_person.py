import time
import dlib
import rospy
import cv2
import gender_predict
import correct_orientation
import numpy as np
from geometry_msgs.msg import Twist


class reach_person:
    def __init__(self, session):
        self.session = session
        # Initiate Service
        self.SoundLoc= self.session.service("ALSoundLocalization")
        self.motion = self.session.service("ALMotion")
        self.Memo = self.session.service("ALMemory")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.if_stop = False
        self.detector = dlib.get_frontal_face_detector()
        self.RobotPos.goToPosture("Stand", .5)
        self.get_image_switch = True
        # Initiate Parameters
        self.width = 640
        self.height = 480
        self.count = 0
        self.center_lengthwise = 0
        self.center_crosswise = 0
        self.face_height = 0
        self.head_angle = -.2
        # Initial switches
        self.switch = True
        self.if_correct = False
        self.first_save = True
        # Initial Ros publisher and subscriber
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Functions
        self.sound_localization()
        while True:
            if self.count >= 3:
                self.stop_loc()
                break

    # Define Functions
    def sound_localization(self):
        self.SoundLoc.subscribe("SoundLocated")
        self.SoundLoc.setParameter("Sensitivity", 0.7)
        self.sound_localization_sub = self.Memo.subscriber("ALSoundLocalization/SoundLocated")
        self.sound_localization_sub.signal.connect(self.callback_sound_localization)

    def stop_loc(self):
        self.SoundLoc.unsubscribe("SoundLocated")
        self.find_person()
        print "Topic unsubscribe!"

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

    def find_person(self):
        self.motion.setAngles("HeadPitch", self.head_angle, .05)
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
                if self.first_save:
                    cv2.imwrite("./capture.jpg", image)
                    gender_predict.gender("./capture.jpg")
                    self.first_save = False
                cv2.waitKey(1)
                self.reach(image)
        return "success"

    def reach(self, frame):
        frame_copy = frame.copy()
        rects = self.detector(frame_copy, 2)
        if len(rects) != 0:
            image_max = 0
            for rect in rects:
                cv2.rectangle(frame_copy, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (0, 0, 255), 2, 8)
                cv2.imshow("yess", frame_copy)
                cv2.imwrite("./person.jpg", frame_copy)
                if (rect.right() - rect.left())*(rect.bottom() - rect.top()) > image_max:
                    image_max = (rect.right() - rect.left()) * (rect.bottom() - rect.top())
                    self.center_lengthwise = (rect.top() + rect.bottom()) / 2
                    self.center_crosswise = (rect.left() + rect.right()) / 2
                    self.face_height = abs(rect.top() - rect.bottom())
                    print "face_height:", self.face_height
            face_ratio = float(image_max) / float(self.width * self.height)
            print 'face ratio:', face_ratio
            print '-----------------------'
            if face_ratio < .005:
                if not self.if_correct:
                    CO = correct_orientation.correct_orientation(self.session, 10)
                    print "Remote correction completed! The error is :", CO.error
                    self.if_correct = True
                self.set_velocity(0.15, 0, 0)
                # Correct orientation
                if abs(self.center_crosswise - self.width / 2) > self.width / 10:
                    self.set_velocity(0, 0, 0.001 * (self.width / 2 - self.center_crosswise))
                    time.sleep(2)
                    self.set_velocity(0.15, 0, 0)
                # Correct head position
                if self.center_lengthwise < self.height / 2:
                    self.head_angle -= .1
                    self.motion.setAngles("Head", [0, self.head_angle], .05)
                elif self.center_lengthwise > (2 * self.height / 3):
                    self.head_angle += .1
                    self.motion.setAngles("Head", [0, self.head_angle], .05)
                cv2.waitKey(1)
            else:
                if not self.if_correct:
                    CO = correct_orientation.correct_orientation(self.session, 50)
                    print "Near correction completed! The error is :", CO.error
                    self.if_correct = True
                if face_ratio > .03:
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
                        self.motion.setAngles("Head", [0, self.head_angle], .05)
                    elif self.center_lengthwise > (2 * self.height / 3):
                        self.head_angle += .1
                        self.motion.setAngles("Head", [0, self.head_angle], .05)
                cv2.waitKey(1)

    # Define Callback Functions
    def callback_sound_localization(self, msg):
        if self.switch == False:
            return
        self.switch = False
        time.sleep(.2)
        self.sound_loc = self.Memo.getData("ALSoundLocalization/SoundLocated")
        print("----Located!----", self.sound_loc[1][2])
        print("Energy:", self.sound_loc[1][3])
        if self.sound_loc[1][2] > .3:
            self.motion.moveTo(0, 0, self.sound_loc[1][0])
        self.count += 1
        self.switch = True


def main(session):
    pio = reach_person(session)


if __name__ == "__main__":
    main()
