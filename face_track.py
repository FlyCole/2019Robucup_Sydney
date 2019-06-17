import time
import dlib
import numpy as np
import cv2


class face_track:
    def __init__(self, session, person_num):
        self.session = session
        self.person_num = person_num
        self.count = 0

        # Initiate Service
        self.motion = self.session.service("ALMotion")
        self.Memo = self.session.service("ALMemory")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.if_stop = False
        self.detector = dlib.get_frontal_face_detector()
        self.RobotPos.goToPosture("Stand", .5)
        self.get_image_switch = True
        # Initiate Parameters
        self.switch = True

    def find_person(self):
        AL_kQVGA = 1
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)
        # create image
        width = 320
        height = 240
        image = np.zeros((height, width, 3), np.uint8)
        while self.get_image_switch:
            result = self.VideoDev.getImageRemote(nameId)
            if result == None:
                print 'cannot capture.'
            elif result[6] == None:
                print 'no image data string.'
            else:
                values = map(ord, list(str(bytearray(result[6]))))
                i = 0
                for y in range(0, height):
                    for x in range(0, width):
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
        if len(rects) != 0:
            image_max = 0
            max_index = 0
            center = 0
            for rect in rects:
                cv2.rectangle(frame_copy, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (0, 0, 255), 2, 8)
                cv2.imshow("track", frame_copy)
                cv2.imwrite("./track.jpg", frame_copy)
                if (rect.right() - rect.left()) * (rect.bottom() - rect.top()) > image_max:
                    image_max = (rect.right() - rect.left()) * (rect.bottom() - rect.top())
                    center = (rect.left() + rect.right()) / 2
            print float(image_max) / float(320 * 410)
            if float(image_max) / float(320 * 410) > .018:
                self.get_image_switch = False
                self.if_stop = True
            else:
                self.motion.moveTo(0.2, 0, 0)
            cv2.waitKey(1)