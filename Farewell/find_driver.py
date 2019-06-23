import cv2
import dlib
import time
import colorsys
import collections
import numpy as np
import PIL.Image as Image


class find_driver:
    def __init__(self, session):
        self.session = session

        # Initiate Service
        self.VideoDev = self.session.service("ALVideoDevice")
        self.Motion = self.session.service("ALMotion")
        # Initiate Parameters
        self.person_num = 0
        self.get_image_switch = True
        self.maxth = 0.12
        self.if_find_driver = False
        self.detector = dlib.get_frontal_face_detector()
        self.width = 640
        self.height = 480

        # Start finding
        self.find()

    # Define Functions
    def find(self):
        # print "------------------------------------------------------"
        AL_kQVGA = 2
        # Need to add All color space variables
        AL_kRGBColorSpace = 13
        fps = 60
        nameId = self.VideoDev.subscribe("image" + str(time.time()), AL_kQVGA, AL_kRGBColorSpace, fps)
        # create image
        image = np.zeros((self.height, self.width, 3), np.uint8)
        # while self.get_image_switch:
        result = self.VideoDev.getImageRemote(nameId)
        if result == None:
            print 'Cannot capture!'
        elif result[6] == None:
            print 'No image data string.'
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
            self.find_significant_cloth(image)
        return self.if_find_driver

    def RGB_detection(self, frame):
        frame_copy = frame.copy()
        self.get_color(frame_copy)

    def getColorList(self):
        dict = collections.defaultdict(list)
        # Black
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 46])
        color_list = []
        color_list.append(lower_black)
        color_list.append(upper_black)
        dict['black'] = color_list
        # Gray
        lower_gray = np.array([0, 0, 46])
        upper_gray = np.array([180, 43, 220])
        color_list = []
        color_list.append(lower_gray)
        color_list.append(upper_gray)
        dict['gray']=color_list
        # White
        lower_white = np.array([0, 0, 221])
        upper_white = np.array([180, 30, 255])
        color_list = []
        color_list.append(lower_white)
        color_list.append(upper_white)
        dict['white'] = color_list
        # Red
        lower_red = np.array([156, 43, 46])
        upper_red = np.array([180, 255, 255])
        color_list = []
        color_list.append(lower_red)
        color_list.append(upper_red)
        dict['red'] = color_list
        # Red2
        lower_red = np.array([0, 43, 46])
        upper_red = np.array([10, 255, 255])
        color_list = []
        color_list.append(lower_red)
        color_list.append(upper_red)
        dict['red2'] = color_list
        # Orange
        lower_orange = np.array([11, 43, 46])
        upper_orange = np.array([25, 255, 255])
        color_list = []
        color_list.append(lower_orange)
        color_list.append(upper_orange)
        dict['orange'] = color_list
        # Yellow
        lower_yellow = np.array([26, 43, 46])
        upper_yellow = np.array([34, 255, 255])
        color_list = []
        color_list.append(lower_yellow)
        color_list.append(upper_yellow)
        dict['yellow'] = color_list
        # Green
        lower_green = np.array([35, 43, 46])
        upper_green = np.array([77, 255, 255])
        color_list = []
        color_list.append(lower_green)
        color_list.append(upper_green)
        dict['green'] = color_list
        # Cyan
        lower_cyan = np.array([78, 43, 46])
        upper_cyan = np.array([99, 255, 255])
        color_list = []
        color_list.append(lower_cyan)
        color_list.append(upper_cyan)
        dict['cyan'] = color_list
        # Blue
        lower_blue = np.array([100, 43, 46])
        upper_blue = np.array([124, 255, 255])
        color_list = []
        color_list.append(lower_blue)
        color_list.append(upper_blue)
        dict['blue'] = color_list
        # Purple
        lower_purple = np.array([125, 43, 46])
        upper_purple = np.array([155, 255, 255])
        color_list = []
        color_list.append(lower_purple)
        color_list.append(upper_purple)
        dict['purple'] = color_list
        return dict

    def get_color(self, frame):
        print('Getting color')
        max = np.size(frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color = None
        color_dict = self.getColorList()
        mask = cv2.inRange(hsv, color_dict["red"][0], color_dict["red"][1])
        cv2.imwrite("red" + '.jpg', mask)
        binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)[1]
        binary = cv2.dilate(binary, None, iterations=2)
        img, cnts, hiera = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        sum = 0
        for c in cnts:
            sum += cv2.contourArea(c)
        print sum / max
        print "-----------"
        if sum / max > self.maxth:
            self.if_find_driver = True
            self.get_image_switch = False
            print "max_ratio:", sum / max
            # self.get_image_switch = False

    def find_significant_cloth(self, frame):
        frame_copy = frame.copy()
        rects = self.detector(frame_copy, 2)
        if len(rects) != 0:
            image_max = 0
            for rect in rects:
                self.person_num += 1
                width = rect.right() - rect.left()
                height = rect.bottom() - rect.top()
                center = (rect.right() + rect.left()) / 2
                cv2.rectangle(frame_copy, (rect.left() - width, rect.bottom()), (rect.right() + width, rect.bottom() + 3 * height), (0, 0, 255), 2, 8)
                cv2.imshow("cloth", frame_copy)
                cv2.imwrite("./cloth.jpg", frame_copy)
                if rect.bottom() + 3 * height > self.height:
                    cut_bottom = self.height
                else:
                    cut_bottom = rect.bottom() + 3 * height
                if rect.left() - width < 0:
                    cut_left = 0
                else:
                    cut_left = rect.left() - width
                if rect.right() + width > self.width:
                    cut_right = self.width
                else:
                    cut_right = rect.right() + width
                RGB_detect = frame_copy[rect.bottom():cut_bottom, cut_left:cut_right]
                self.RGB_detection(RGB_detect)

    def get_person_num(self):
        return self.person_num