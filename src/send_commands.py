#!/usr/bin/env python2

import rospy
from std_msgs.msg import String, Int8, Int16, Int32
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk

import os

os.environ['ROS_MASTER_URI']='http://192.168.1.247:11311'

rospy.init_node("robobo__demo")

robobo_move_srv = rospy.ServiceProxy('robot/moveWheels', MoveWheels)
robobo_emotion_srv = rospy.ServiceProxy('robot/setEmotion', SetEmotion)
robobo_talk_srv = rospy.ServiceProxy('robot/talk', Talk)

robobo_emotion_srv(String('sad'))
robobo_move_srv(Int8(20), Int8(-20), Int32(2000), Int16(0))
robobo_talk_srv(String('Hi, my name is Robobo'))
rospy.sleep(1)
robobo_move_srv(Int8(-20), Int8(20), Int32(2000), Int16(0))
robobo_emotion_srv(String('happy'))