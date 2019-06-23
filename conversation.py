#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import os
import dlib
import json
import base64
import requests
import time
from threading import Thread

class conversation:
    def __init__(self, session, ip):
        self.session = session
        self.ip = ip
        # Initiate Services
        self.Leds = self.session.service("ALLeds")
        self.Memo = self.session.service("ALMemory")
        self.AudioDev = self.session.service("ALAudioDevice")
        self.AudioPla = self.session.service("ALAudioPlayer")
        self.AudioRec = self.session.service("ALAudioRecorder")
        self.TextToSpe = self.session.service("ALTextToSpeech")
        self.if_stop = False
        self.detector = dlib.get_frontal_face_detector()
        self.get_image_switch = True
        # Initiate Parameters
        self.if_correct = False
        self.center = 0
        self.error = 0
        self.width = 640
        self.height = 480
        self.RATE = "16000"
        self.FORMAT = "wav"
        self.CUID = "wave_play"
        self.DEV_PID = 1737
        self.framerate = 16000
        self.NUM_SAMPLES = 2000
        self.beep_volume = 70
        self.channels = 1
        self.sampwidth = 2
        self.TIME = 2
        self.size = None
        self.speech = None
        self.speech_hints = []
        self.recog_result = None
        self.thread_recording = Thread(target=self.record_audio, args=(None,))
        self.thread_recording.daemon = True
        self.audio_terminate = False
        self.name_list = ["Alex", "Charlie", "Elizabeth", "Francis", "James", "Jennifer", "John", "Linda", "Michael",
                          "Mary", "Robert", "Patricia", "Robin", "Skyler", "William"]
        self.led_name = ["Face/Led/Blue/Right/0Deg/Actuator/Value", "Face/Led/Blue/Right/45Deg/Actuator/Value",
                         "Face/Led/Blue/Right/90Deg/Actuator/Value", "Face/Led/Blue/Right/135Deg/Actuator/Value",
                         "Face/Led/Blue/Right/180Deg/Actuator/Value", "Face/Led/Blue/Right/225Deg/Actuator/Value",
                         "Face/Led/Blue/Right/270Deg/Actuator/Value", "Face/Led/Blue/Right/315Deg/Actuator/Value",
                         "Face/Led/Blue/Left/0Deg/Actuator/Value", "Face/Led/Blue/Left/45Deg/Actuator/Value",
                         "Face/Led/Blue/Left/90Deg/Actuator/Value", "Face/Led/Blue/Left/135Deg/Actuator/Value",
                         "Face/Led/Blue/Left/180Deg/Actuator/Value", "Face/Led/Blue/Left/225Deg/Actuator/Value",
                         "Face/Led/Blue/Left/270Deg/Actuator/Value", "Face/Led/Blue/Left/315Deg/Actuator/Value"]
        self.Leds.createGroup("MyGroup", self.led_name)
        self.if_need_record = False
        # Functions
        self.start_recording(reset=True)

    def start_recording(self, reset = False, base_duration = 3, withBeep = True):
        self.if_need_record = True
        self.record_time = time.time() + base_duration
        if reset:
            self.kill_recording_thread()
            self.AudioRec.stopMicrophonesRecording()
        print "self.record_time is:  ", self.record_time
        if not self.thread_recording.is_alive():
            self.thread_recording = Thread(target=self.record_audio, args=(self.speech_hints, withBeep))
            self.thread_recording.daemon = False
            self.thread_recording.start()
            self.thread_recording.join()
            print('\033[0;32m [Kamerider I] Start recording thread \033[0m')

    def kill_recording_thread(self):
        if self.thread_recording.is_alive():
            self.audio_terminate = True
            time.sleep(0.3)
            self.audio_terminate = False

    def record_audio(self, hints, withBeep=True):
        # Turn on the light
        if withBeep:
            # Parameters playSine(const int& frequence, const int& gain, const int& pan, const float& duration)
            self.AudioPla.playSine(1000, self.beep_volume, 1, .3)
        print('\033[0;32m [Kamerider I] start recording... \033[0m')
        channels = [0, 0, 1, 0]
        self.AudioRec.startMicrophonesRecording("/home/nao/audio/recog.wav", "wav", 16000, channels)
        while time.time() < self.record_time:
            if self.audio_terminate:
                # If stop:True
                self.AudioRec.stopMicrophonesRecording()
                print('\033[0;32m [Kamerider I] Killing recording... \033[0m')
                return None
            time.sleep(.1)
        self.Leds.off("MyGroup")
        self.AudioRec.stopMicrophonesRecording()
        self.AudioRec.recording_ended = True
        if not os.path.exists('./audio_record'):
            os.mkdir('./audio_record', 0o755)
        # Copy the recorded audio to computer
        cmd = 'sshpass -p kurakura326 scp nao@' + str(self.ip) + ":/home/nao/audio/recog.wav ./audio_record"
        os.system(cmd)
        print('\033[0;32m [Kamerider I] Record ended start recognizing \033[0m')
        self.recog_result = self.baidu_recognition("./audio_record/recog.wav").lower()
        print "===============", self.recog_result

    def baidu_recognition(self, path):
        token = self.get_token()
        try:
            ret = self.get_word(token, path)
            result = str(ret)
        except:
            print('Failed')
            return "00"
        print result
        return result

    def get_token(self):
        server = "https://openapi.baidu.com/oauth/2.0/token?"
        grant_type = "client_credentials"
        # API Key
        client_id = "2cCSUMGb9BtO7eWLOxjxnod5"
        # Secret Key
        client_secret = "Gu7UdTeCPVH7h76G5P4MpyMpgviPomaG"
        # url
        url = "%sgrant_type=%s&client_id=%s&client_secret=%s" % (server, grant_type, client_id, client_secret)
        # Obtain token
        res = requests.post(url)
        token = json.loads(res.text)["access_token"]
        # print token
        return token

    def get_word(self, token, path):
        with open(path, "rb") as f:
            self.speech = base64.b64encode(f.read()).decode('utf8')
        self.size = os.path.getsize(path)
        headers = {'Content-Type': 'application/json'}
        url = "https://vop.baidu.com/server_api"
        data = {
            "format":self.FORMAT,
            "rate":self.RATE,
            "dev_pid": self.DEV_PID,
            "speech": self.speech,
            "cuid": self.CUID,
            "len": self.size,
            "channel": self.channels,
            "token": token,
        }
        req = requests.post(url, json.dumps(data), headers)
        try:
            result = json.loads(req.text)
        except Exception as e:
            print e
            return "none"
        return result["result"][0]

    def get_result(self):
        return self.recog_result