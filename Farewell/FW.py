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
import re
import sys
import time
import rospy
import atexit
import actionlib
import reach_person
import find_driver
import face_track
import conversation
import thread
import tf
import correct_orientation
import speech_recognization_FW
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalID
from math import pi
from tf_conversions import transformations
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
        self.point_added = 0
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
        self.head_fix = False
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
        self.keyboard_control()


    # Functions
    def __del__(self):
        print ('\033[0;32m [Kamerider I] System Shutting Down... \033[0m')
        self.AudioRec.stopMicrophonesRecording()
        self.Tracker.stopTracker()
        self.Tracker.unregisterAllTargets()

    def start(self):
        self.start_head_fix()
        self.find_person()
        self.catch_cloth()
        self.go_out()
        self.find_driver()
        self.reach_driver()
        self.go_back()

    def find_person(self):
        self.TextToSpe.say("I'm ready to carry out this mission.")
        self.angle = -.35
        reach_person.main(self.session)
        self.save_point()

    def catch_cloth(self):
        self.TextToSpe.say("May I have your name, sir?")
        clothes = {
            "damn": "point18",
            "ben": "point22",
            "cat": "point15",
            "mary": "point14",
            "alex": "point7"
        }
        cloth_colors = {
            "damn": "red",
            "ben": "yellow",
            "cat": "green",
            "mary": "black",
            "alex": "blue"
        }
        # osr = speech_recognization_FW.offline_speech_recognization(self.session)
        while True:
            con = conversation.conversation(self.session, self.ip)
            con_result = con.get_result()
            print "The results of Baidu is:", con_result
            if con_result != "00":
                point = self.audio_analyze(con_result, clothes)
                print point
                self.head_fix = True
                saying = "I can't take the coat. Could you please follow me and take your " + str(cloth_colors[con_result]) + " coat."
                self.TextToSpe.say(saying)
                self.go_to_waypoint(self.point_dataset["point15"], "point2")
                self.go_to_waypoint(self.point_dataset[point], "point1")
                saying = "Here's your " + str(cloth_colors[con_result]) + " coat."
                self.TextToSpe.say(saying)
                self.TextToSpe.say("Please follow me. I will guide you to the driver.")
                break

    def find_driver(self):
        self.angle = -.3
        self.Motion.setAngles("Head", [0., self.angle], .05)
        self.TextToSpe.say("I'm ready to find the driver!")
        while True:
            self.Motion.moveTo(0, 0, 3.14 / 6)
            FD = find_driver.find_driver(self.session)
            if FD.if_find_driver:
                break
        CO = correct_orientation.correct_orientation(self.session, 50)
        print "error:", CO.error
        self.TextToSpe.say("I have already found the driver! Please follow me!")

    def go_out(self):
        self.go_to_waypoint(self.point_dataset["point31"], "point1")
        self.go_to_waypoint(self.point_dataset["point7"], "point1")
        self.go_to_waypoint(self.point_dataset["point5"], "point2")
        self.head_fix = False

    def go_back(self):
        self.head_fix = True
        self.go_to_waypoint(self.point_dataset["point6"], "point3")
        self.go_to_waypoint(self.point_dataset["point8"], "point4")
        self.go_to_waypoint(self.point_dataset["point13"], "point5")
        self.go_to_waypoint(self.point_dataset["point16"], "point6")
        self.go_to_waypoint(self.point_dataset["point19"], "point7")

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

    def save_point(self):
        self.if_save_switch = True
        print "save_point"
        # amcl定位
        amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        while self.if_save_switch:
            time.sleep(1)
            amcl_sub.unregister()

    def amcl_callback(self, msg):
        point_temp = MoveBaseGoal()
        point_temp.target_pose.header.frame_id = '/map'
        point_temp.target_pose.header.stamp = msg.header.stamp
        point_temp.target_pose.header.seq = msg.header.seq
        point_temp.target_pose.pose = msg.pose.pose
        self.point_dataset.update({"point_add" + str(self.point_added): point_temp})
        print('\033[0;32m [Kamerider I] Point saved successfully!! \033[0m')
        self.if_save_switch = False

    def head_fix_thread(self, arg):
        self.Motion.setStiffnesses("head", 1.0)
        while True:
            if self.head_fix:
                # print "=====self.angle:====", self.angle
                self.Motion.setAngles("Head", [0., self.angle], .2)
            time.sleep(3)

    def start_head_fix(self):
        arg = tuple([1])
        thread.start_new_thread(self.head_fix_thread, arg)

    def set_velocity(self, x, y, theta, duration=-1.):  # m/sec, rad/sec
        # if duration > 0 : stop after duration(sec)
        tt = Twist()
        tt.linear.x = x
        tt.linear.y = y
        tt.angular.z = theta
        self.cmd_vel_pub.publish(tt)
        if duration < 0: return None
        tic = time.time()
        while time.time() - tic < duration:
            self.cmd_vel_pub.publish(tt)
            time.sleep(0.1)
        tt = Twist()
        tt.linear.x = 0
        tt.linear.y = 0
        tt.angular.z = 0
        self.cmd_vel_pub.publish(tt)

    def stop_motion(self):
        self.set_velocity(0, 0, 0)

    def keyboard_control(self):
        print('\033[0;32m [Kamerider I] Start keyboard control \033[0m')
        command = ''
        while command != 'c':
            try:
                command = raw_input('next command : ')
                if command == 'st':
                    self.start()
                elif command == 'w':
                    self.set_velocity(0.25, 0, 0)
                elif command == 's':
                    self.stop_motion()
                elif command == 'x':
                    self.set_velocity(-0.25, 0, 0)
                elif command == 'a':
                    self.set_velocity(0, 0.25, 0)
                elif command == 'd':
                    self.set_velocity(0, -0.25, 0)
                elif command == 'q':
                    self.set_velocity(0, 0, 0.35)
                elif command == 'e':
                    self.set_velocity(0, 0, -0.35)
                elif command == "print":
                    print self.point_dataset
                elif command == 'c':
                    break
                else:
                    print("Invalid Command!")
            except Exception as e:
                print e

def main():
    params = {
        'ip': "192.168.3.93",
        'port': 9559,
        'rgb_topic': 'pepper_robot/camera/front/image_raw'
    }
    Farewell(params)


if __name__ == "__main__":
    main()