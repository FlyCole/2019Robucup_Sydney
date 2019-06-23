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
import reach_person
import find_driver
import face_track
import conversation
import correct_orientation
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
        self.if_find_driver = False
        self.if_correct = False
        self.current_person_name = "None"
        self.point_dataset = self.load_waypoint("waypoints_FW.txt")

        # ROS
        self.nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.nav_as.wait_for_server()
        # Clear costmap
        self.map_clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.map_clear_srv()
        self.Motion.setTangentialSecurityDistance(.05)
        self.Motion.setOrthogonalSecurityDistance(.1)

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
        self.person_name = ["alex", "mary", "david"]
        # TimeStamp
        ticks = time.time()
        # 0 represents TOP camera, the last parameter is FPS
        self.VideoDev.subscribeCamera(str(ticks), 0, 2, 11, 40)
        # set dialog language
        self.Dialog.setLanguage("English")
        self.TextToSpe.setLanguage("English")
        # the path of the recorded audio
        self.audio_path = '/home/nao/audio/record.wav'
        self.recog_result = "None"
        # Beep volume
        self.beep_volume = 70
        self.head_fix = True
        # initial the position of head, fix the position when walking
        self.Motion.setStiffnesses("Head", 1.0)
        self.Motion.setAngles("Head", [0., self.angle], .05)
        # set the speed of speaking
        self.TextToSpe.setParameter("speed", 80.0)
        # Close AutonomousLife Model
        if self.AutonomousLife.getState() != "disabled":
            self.AutonomousLife.setState("disabled")
        self.RobotPos.goToPosture("Stand", .5)

        # Invoke Member-Function
        self.find_person()
        self.catch_cloth()
        # self.go_out()
        self.find_driver()
        self.reach_driver()

        self.keyboard_control()

    # Functions
    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')
        self.AudioRec.stopMicrophonesRecording()
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()

    def find_person(self):
        self.TextToSpe.say("I'm ready to carry out this mission.")
        self.angle = -.35
        reach_person.main(self.session)

    def catch_cloth(self):
        self.TextToSpe.say("May i have your name, sir?")
        
        clothes = {
            "Zhangjiashi": "point1",
            "Renyifei": "point2",
            "Lindasheng": "point3",
            "Xuyucheng": "point4",
            "alex": "point5"
        }
        while True:
            con = conversation.conversation(self.session, self.ip)
            con_result = con.get_result()
            print "The results of Baidu is:", con_result
            if con_result != "00":
                point = self.audio_analyze(con_result, clothes)
                print point
                # self.go_to_waypoint(self.point_dataset[point], "point1")
                self.TextToSpe.say("Wait a minute. I will guide you to the cab, don't forget your coat!")
                break



    def find_driver(self):
        self.TextToSpe.say("I'm ready to find the driver!")
        while True:
            self.Motion.moveTo(0, 0, 3.14 / 6)
            FD = find_driver.find_driver(self.session)
            if FD.if_find_driver:
                break
        CO = correct_orientation.correct_orientation(self.session)
        print "error:", CO.error
        self.TextToSpe.say("I have already found the driver! Please follow me!")

    def go_out(self):
        #self.go_to_waypoint(self.point_dataset["point19"], "point1", "first")
        print 1

    def reach_driver(self):
        FT = face_track.face_track(self.session)
        FT.find_person()
        self.TextToSpe.say("Here's the driver. Have a good night!")

    def load_waypoint(self, file_name):
        curr_pos = PoseStamped()
        f = open(file_name, 'r')
        sourceInLines = f.readlines()
        dataset_points = {}
        for line in sourceInLines:
            temp1 = line.strip('\n')
            temp2 = temp1.split(',')
            point_temp = MoveBaseGoal()
            point_temp.target_pose.header.frame_id = '/map'
            point_temp.target_pose.header.stamp = curr_pos.header.stamp
            point_temp.target_pose.header.seq = curr_pos.header.seq
            point_temp.target_pose.pose.position.x = float(temp2[1])
            point_temp.target_pose.pose.position.y = float(temp2[2])
            point_temp.target_pose.pose.position.z = float(temp2[3])
            point_temp.target_pose.pose.orientation.x = float(temp2[4])
            point_temp.target_pose.pose.orientation.y = float(temp2[5])
            point_temp.target_pose.pose.orientation.z = float(temp2[6])
            point_temp.target_pose.pose.orientation.w = float(temp2[7])
            dataset_points[temp2[0]] = point_temp
        print ("↓↓↓↓↓↓↓↓↓↓↓↓point↓↓↓↓↓↓↓↓↓↓↓↓")
        print (dataset_points)
        print ("↑↑↑↑↑↑↑↑↑↑↑↑point↑↑↑↑↑↑↑↑↑↑↑↑")
        print ('\033[0;32m [Kamerider I] Points Loaded! \033[0m')
        return dataset_points

    def go_to_waypoint(self, Point, label="none"):  # Point代表目标点 destination代表目标点的文本 label
        self.angle = .3
        self.nav_as.send_goal(Point)
        self.map_clear_srv()
        count_time = 0
        # = 3, when Pepper reaches the destination
        while self.nav_as.get_state() != 3:
            count_time += 1
            time.sleep(1)
            if count_time == 8:
                self.map_clear_srv()
                count_time = 0
        # self.TextToSpe.say("I have arrived at " + destination)
        if label == "none":
            return

    def audio_analyze(self, result, clothes):
        point = None
        for i in range(len(self.person_name)):
            if re.search(self.person_name[i], result) != None:
                point = clothes[self.person_name[i]]
        return point

    def keyboard_control(self):
        print('\033[0;32m [Kamerider I] Start keyboard control \033[0m')
        command = ''
        while command != 'c':
            try:
                command = raw_input('next command : ')
                if command == 'w':
                    break
                else:
                    print("Invalid Command!")
            except EOFError:
                print "Error!!"


def main():
    params = {
        'ip': "192.168.3.93",
        'port': 9559,
        'rgb_topic': 'pepper_robot/camera/front/image_raw'
    }
    Farewell(params)


if __name__ == "__main__":
    main()