from __future__ import unicode_literals, print_function, absolute_import, division, generators, nested_scopes
from robobo.base import Robobo

import os
import time
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int8, Int16, Int32
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk, SetLed
from robobo_msgs.msg import IRs
from sensor_msgs.msg import CompressedImage

class HardwareRoboboException(Exception):
    pass

class HardwareRobobo(Robobo):
    # ROS Topics
    MOVE_WHEELS_TOPIC = 'robot/moveWheels'
    SET_EMOTION_TOPIC = 'robot/setEmotion'
    TALK_TOPIC        = 'robot/talk'
    SET_LED_TOPIC     = 'robot/setLed'
    IMAGE_TOPIC       = 'robot/camera/image/compressed'

    def __init__(self, camera=False):
        self._uri = None
        self._enable_camera = camera

    def connect(self, address, port=11311):
        self._uri = "http://{}:{}".format(address, port)
        self._set_env()

        rospy.init_node("robobo_demo")
        self._move_srv        = rospy.ServiceProxy(HardwareRobobo.MOVE_WHEELS_TOPIC, MoveWheels)
        self._emotion_srv     = rospy.ServiceProxy(HardwareRobobo.SET_EMOTION_TOPIC, SetEmotion)
        self._talk_srv        = rospy.ServiceProxy(HardwareRobobo.TALK_TOPIC,        Talk)
        self._leds_srv        = rospy.ServiceProxy(HardwareRobobo.SET_LED_TOPIC,     SetLed)
        if self._enable_camera:
            self._image_subscribe_front = rospy.Subscriber(HardwareRobobo.IMAGE_TOPIC, CompressedImage, self._camera_callback_front, queue_size=1)
            self._receiving_image_front = None
        return self

    def _set_env(self):
        os.environ['ROS_MASTER_URI'] = self._uri

    def spin(self):
        self._set_env()
        rospy.spin()

    def set_emotion(self, emotion):
        self._set_env()
        self._emotion_srv(String(emotion))

    def move(self, left, right, millis, blockid=0):
        self._set_env()
        self._move_srv(Int8(left), Int8(right), Int32(millis), Int16(blockid))
        time.sleep(millis/1000.0)
    
    def talk(self, message):
        self._set_env()
        self._talk_srv(String(message))

    def set_led(self, selector, color):
        self._set_env()
        self._leds(String(selector), String(color))
    
    def set_irs_listener(self, listener):
        self._set_env()
        self._IRsub = rospy.Subscriber("robot/irs", IRs, listener)

    def _camera_callback_front(self, ros_data):
        if self._receiving_image_front is None:
            #### direct conversion to CV2 ####
            np_arr = np.fromstring(ros_data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # save result in memory (FLIP NEEDED [only front camera?])
            self._receiving_image_front = cv2.flip(image_np, 1)

    def get_image_front(self):
        if not self._enable_camera:
            raise HardwareRoboboException("Camera is disabled")
        # Retrieve last image from image topic
        #import datetime
        #start = datetime.datetime.now()
        self._receiving_image_front = None
        while self._receiving_image_front is None:
            time.sleep(0.002)
        #print("Image download took {}".format(datetime.datetime.now()-start))
        image = self._receiving_image_front
        self._receiving_image_front = None
        

        return image

    def sleep(self, seconds):
        self._set_env()
        rospy.sleep(seconds)
