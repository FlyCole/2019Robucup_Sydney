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


def capture_pic():
    cap = cv2.VideoCapture(0)
    i = 0
    while (1):
        ret, frame = cap.read()
        k = cv2.waitKey(1)
        if k == 27:
            break
        elif k == ord('s'):
            cv2.imwrite('capture.jpg', frame)
            i += 1
        cv2.imshow("capture", frame)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    capture_pic()