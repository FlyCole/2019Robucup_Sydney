#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
"""
    Author: Yifei Ren
    Name: Farewell
    Version: 1.0
    Date: 03/06/2019
    Description: Guide two people who are tired to the cab carrying their cloths.
    Note: Take one person each time.
"""
import qi
import os
import re
import cv2
import sys
import time
import rospy
import thread
import atexit
import actionlib
import numpy as np
import vision_definitions
from threading import Thread
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped


class Farewell():
    def __init__(self, params):
        atexit.register(self.__del__)
        # pepper connection
        self.ip = params["ip"]
        self.port = params["port"]
        self.session = qi.Session()
        rospy.init_node("Farewell")
        # Try to connect to pepper
        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("[Kamerider E] : connection Error!!")
            sys.exit(1)

        # naoqi API
        self.Leds = self.session.service("ALLeds")
        self.Memory = self.session.service("ALMemory")
        self.Dialog = self.session.service("ALDialog")
        self.Motion = self.session.service("ALMotion")
        self.Tracker = self.session.service("ALTracker")
        self.VideoDev = self.session.service("ALVideoDevice")
        self.AudioDev = self.session.service("ALAudioDevice")
        self.AudioPla = self.session.service("ALAudioPlayer")
        self.RobotPos = self.session.service("ALRobotPosture")
        self.AudioRec = self.session.service("ALAudioRecorder")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.BasicAwa = self.session.service("ALBasicAwareness")
        self.TabletSer = self.session.service("ALTabletService")
        self.SoundDet = self.session.service("ALSoundDetection")
        self.SoundLoc = self.session.service("ALSoundLocalization")
        self.AnimatedSpe = self.session.service("ALAnimatedSpeech")
        self.AutonomousLife = self.session.service("ALAutonomousLife")

        # Set parameters
        self.sound_switch = True
        self.angle = -0.4

        # Close basic_awareness
        if self.BasicAwa.isEnabled():
            self.BasicAwa.setEnabled(False)
        if self.BasicAwa.isRunning():
            self.BasicAwa.pauseAwareness()
        # Initiate tablet
        self.TabletSer.cleanWebview()
        print ('\033[0;32m [Kamerider I] Tablet initialize successfully \033[0m')

        # Initiate key words
        self.place = ["bathroom", "kitchen", "party room", "dining room","dinning room", "living room", "bedroom"]
        self.start = ["follow", "following", "start", "follow me"]
        self.stop = ["stop", "here is the car"]
        self.go_back = ["bathroom", "living room", "bedroom", "kitchen", "toilet"]
        # TimeStamp
        ticks = time.time()
        # 0代表top相机 最后一个参数是fps
        self.VideoDev.subscribeCamera(str(ticks), 0, 2, 11, 40)
        # 设置dialog语言
        self.Dialog.setLanguage("English")
        self.TextToSpe.setLanguage("English")
        # 录下的音频保存的路径
        self.audio_path = '/home/nao/audio/record.wav'
        self.recog_result = "None"
        # Beep 音量
        self.beep_volume = 70
        self.head_fix = True
        # 设置初始化头的位置 走路的时候固定头的位置
        self.Motion.setStiffnesses("Head", 1.0)
        self.Motion.setAngles("Head", [0., self.angle], .05)
        # 设置说话速度
        self.TextToSpe.setParameter("speed", 80.0)
        # 关闭AutonomousLife模式
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("Stand", .5)

        # Invoke Member-Function
        self.sound_localization()


    # Functions
    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')
        self.AudioRec.stopMicrophonesRecording()
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()


    def sound_localization(self):
        self.SoundLoc.subscribe("SoundLocated")
        self.SoundLoc.setParameter("Sensitivity", 0.7)
        self.sound_localization_sub = self.Memory.subscriber("ALSoundLocalization/SoundLocated")
        self.sound_localization_sub.signal.connect(self.callback_localization)


    # Callback Functions
    def callback_localization(self, msg):
        if self.sound_switch == False:
            return
        self.sound_switch = False
        sound_loc = self.Memory.getData("ALSoundLocalization/SoundLocated")
        print "----Sound located!----", sound_loc[1][2]
        print "Energy:", sound_loc[1][3]
        if sound_loc[1][2] > .5:
            self.Motion.moveTo(0, 0, sound_loc[1][0])
        self.sound_switch = True


def main():
    params = {
        'ip' : "192.168.3.93",
        'port' : 9559,
        'rgb_topic' : 'pepper_robot/camera/front/image_raw'
    }
    Farewell(params)


if __name__ == "__main__":
    main()