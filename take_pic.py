# -*- encoding: UTF-8 -*-
import qi
import atexit
import time
class PhotoCapture:
    def __init__(self):
        atexit.register(self.__del__)
        self.tp = session.service("ALPhotoCapture")
        # self.tts = session.service("ALTextToSpeech")

        # self.tts.setParameter("speed", 75.0)
        # self.tts.setLanguage("English")
        print("ready to take pictures")

    def takepic(self):
        # self.tts.say("I'm going to take a picture")
        self.tp.setResolution(3)
        self.tp.takePictures(3, '/home/nao/picture', 'image')
        # self.tts.say("picture's taken")
        print("picture's taken")

    def __del__(self):
        print "exit"


if __name__ == '__main__':
    pc = PhotoCapture()
