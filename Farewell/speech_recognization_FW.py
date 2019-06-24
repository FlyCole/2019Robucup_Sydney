#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
"""
    Author: Yifei Ren
    Name: Off-line Speech Recognization
    Version: 1.0
    Date: 23/06/2019
    Description: Off-line speech recognization.
    Note: Using ALDialog Methods.
"""
import qi
import sys
import time

class offline_speech_recognization():
    def __init__(self, session):
        self.session = session
        # Initial Service
        self.Dialog = self.session.service("ALDialog")
        self.Memo = self.session.service("ALMemory")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.Dialog.setLanguage("English")
        # Initial parameter
        self.if_end = False
        # Loading the topic given by the user (absolute path is required)
        topic_path = "/home/nao/topic"
        self.topf_path = topic_path.decode('utf-8')
        self.topic_name = self.Dialog.loadTopic(self.topf_path.encode('utf-8'))
        self.Dialog.activateTopic(self.topic_name)
        # Activating the loaded topic
        self.TextToSpe.say("I'm ready to answer questions.")
        time.sleep(0.5)
        self.Dialog.subscribe('offline_dialog')
        self.end_of_dialog = self.Memo.subscriber("stop_talking")
        self.end_of_dialog.signal.connect(self.callback_stoptalking)
        while not self.if_end:
            print '----', 'dialog lasting', '----'


    def callback_stoptalking(self, msg):
        print msg
        self.Dialog.unsubscribe('offline_dialog')
        self.Dialog.deactivateTopic(self.topic_name)
        self.if_end = True
        self.TextToSpe.say("Thanks for your cooperation")
        # Saying "That's all"