import time
import dlib
import numpy as np
import cv2


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
            center_lengthwise = 0
            center_crosswise = 0
            for rect in rects:
                cv2.rectangle(frame_copy, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (0, 0, 255), 2, 8)
                cv2.imshow("track", frame_copy)
                cv2.imwrite("./track.jpg", frame_copy)
                if (rect.right() - rect.left()) * (rect.bottom() - rect.top()) > image_max:
                    image_max = (rect.right() - rect.left()) * (rect.bottom() - rect.top())
                    center_lengthwise = (rect.bottom() + rect.top()) / 2
                    center_crosswise = (rect.left() + rect.right()) / 2
            print float(image_max) / float(self.height * self.width)
            if float(image_max) / float(self.height * self.width) > .03:
                self.get_image_switch = False
                self.if_stop = True
            else:
                self.Motion.moveTo(0.2, 0, 0)
            if center_lengthwise <= 0.25 * self.height:
                self.Motion.setAngles("HeadPitch", -.2, .05)
            self.Motion.moveTo(0, 0, k * (self.width / 2 - center_crosswise))
            cv2.waitKey(1)