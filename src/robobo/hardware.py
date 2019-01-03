from __future__ import unicode_literals, print_function, absolute_import, division, generators, nested_scopes
from robobo.base import Robobo

import os
import rospy
from std_msgs.msg import String, Int8, Int16, Int32
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk, SetLed
from robobo_msgs.msg import IRs

class HardwareRobobo(Robobo):
    def __init__(self):
        self._uri = ""

    def connect(self, uri):
        if uri is not None:
            self._uri = uri

        os.environ['ROS_MASTER_URI'] = self._uri

        rospy.init_node("robobo_demo")
        self._move_srv    = rospy.ServiceProxy('robot/moveWheels', MoveWheels)
        self._emotion_srv = rospy.ServiceProxy('robot/setEmotion', SetEmotion)
        self._talk_srv    = rospy.ServiceProxy('robot/talk', Talk)
        self._leds_srv    = rospy.ServiceProxy('robot/setLed', SetLed)

    def spin(self):
        rospy.spin()

    def set_emotion(self, emotion):
        self._emotion_srv(String(emotion))

    def move(self, left, right, millis, blockid=0):
        self._move_srv(Int8(left), Int8(right), Int32(millis), Int16(blockid))
    
    def talk(self, message):
        self._talk_srv(String(message))

    def set_led(self, selector, color):
        self._leds(String(selector), String(color))
    
    def set_irs_listener(self, listener):
        self._IRsub = rospy.Subscriber("robot/irs", IRs, listener)

    def sleep(self, seconds):
        rospy.sleep(1)
