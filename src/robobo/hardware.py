from __future__ import unicode_literals, print_function, absolute_import, division, generators, nested_scopes
from robobo.base import Robobo

import os
import time
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int8, Int16, Int32
from robobo_msgs.srv import MoveWheels, MovePanTilt, SetEmotion, Talk, SetLed
from robobo_msgs.msg import IRs
from sensor_msgs.msg import CompressedImage

class HardwareRoboboException(Exception):
    pass

class HardwareRobobo(Robobo):
    # ROS Topics
    MOVE_WHEELS_TOPIC = 'robot/moveWheels'
    PAN_TILT_TOPIC    = 'robot/movePanTilt'
    SET_EMOTION_TOPIC = 'robot/setEmotion'
    TALK_TOPIC        = 'robot/talk'
    SET_LED_TOPIC     = 'robot/setLed'
    IMAGE_TOPIC       = 'robot/camera/image/compressed'

    def __init__(self, camera=False):
        self._uri = None
        self._enable_camera = camera

    def connect(self, address, port=11311):
        self._uri = "http://{}:{}".format(address, port)
        self._irs_values = [0.0 for _ in range(8)]
        self._set_env()

        rospy.init_node("robobo_demo")

        # Service Proxys
        self._move_srv        = rospy.ServiceProxy(HardwareRobobo.MOVE_WHEELS_TOPIC, MoveWheels)
        self._pan_tilt_srv    = rospy.ServiceProxy(HardwareRobobo.PAN_TILT_TOPIC,    MovePanTilt)
        self._emotion_srv     = rospy.ServiceProxy(HardwareRobobo.SET_EMOTION_TOPIC, SetEmotion)
        self._talk_srv        = rospy.ServiceProxy(HardwareRobobo.TALK_TOPIC,        Talk)
        self._leds_srv        = rospy.ServiceProxy(HardwareRobobo.SET_LED_TOPIC,     SetLed)

        # Sensor Receivers
        self._IRsub = rospy.Subscriber("robot/irs", IRs, self._irs_callback)

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

    def move(self, left_speed, right_speed, millis, blockid=0):
        """
        Move the robot wheels for `millis` time

        Arguments

        left_speed: speed of the left wheel. Range: 0-100
        right_speed: speed of the right wheel. Range: 0-100
        millis: how many millisecond to move the robot
        """
        self._set_env()
        self._move_srv(Int8(left_speed), Int8(right_speed), Int32(millis), Int16(blockid))
        time.sleep(millis/1000.0)
    
    def talk(self, message):
        self._set_env()
        self._talk_srv(String(message))

    def set_led(self, selector, color):
        self._set_env()
        self._leds(String(selector), String(color))
    
    def read_irs(self):
        """
        Returns sensor readings: [backR, backC, backL, frontRR, frontR, frontC, frontL, frontLL]
        """
        return self._irs_values

    def _irs_callback(self, ros_data):
        self._irs_values = [
            ros_data.BackR.range,
            ros_data.BackR.range,
            ros_data.BackL.range,
            ros_data.FrontRR.range,
            ros_data.FrontR.range,
            ros_data.FrontC.range,
            ros_data.FrontL.range,
            ros_data.FrontLL.range
        ]
        # print(self._irs_values)

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

    def set_phone_pan(self, pan_position, pan_speed, pan_blockid=1):
        """
        Command the robot to move the smartphone holder in the horizontal (pan) axis.
        This function is asyncronous.

        Arguments

        pan_position: Angle to position the pan at. Range: 11-343.
        pan_speed: Movement speed for the pan mechanism. Range: 0-100.
        pan_blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic. Must be greater that 0 to move.
        """
        self._set_env()
        self._pan_tilt_srv(Int16(pan_position), Int8(pan_speed), Int16(pan_blockid), Int16(0), Int8(0), Int16(0))

    def set_phone_tilt(self, tilt_position, tilt_speed, tilt_blockid=1):
        """
        Command the robot to move the smartphone holder in the vertical (tilt) axis.
        This function is asyncronous.

        Arguments

        tilt_position: Angle to position the tilt at. Range: 26-109.
        tilt_speed: Movement speed for the tilt mechanism. Range: 0-100.
        tilt_blockid: A unique 'blockid' for end-of-movement notification at /robot/unlock/move topic. Must be greater that 0 to move.
        """
        self._set_env()
        self._pan_tilt_srv(Int16(0), Int8(0), Int16(0), Int16(tilt_position), Int8(tilt_speed), Int16(tilt_blockid))

    def sleep(self, seconds):
        self._set_env()
        rospy.sleep(seconds)
