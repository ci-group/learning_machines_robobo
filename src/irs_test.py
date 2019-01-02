#!/usr/bin/env python2

import rospy
import std_msgs.msg
import robobo_msgs.srv
import robobo_msgs.msg
# from std_msgs.msg import String, Int8, Int16, Int32
# from robobo_msgs.srv import MoveWheels, SetEmotion, Talk
# from robobo_msgs.msg import IRs

rospy.init_node("robobo__demo")

# robobo_move_srv    = rospy.ServiceProxy('robot/moveWheels', robobo_msgs.srv.MoveWheels)
# robobo_emotion_srv = rospy.ServiceProxy('robot/setEmotion', robobo_msgs.srv.SetEmotion)
# robobo_talk_srv    = rospy.ServiceProxy('robot/talk', robobo_msgs.srv.Talk)
robobo_leds        = rospy.ServiceProxy('robot/setLed', robobo_msgs.srv.SetLed)

def listener(irs):
    value = irs.FrontC.range
    print("value: {}".format(value))
    robobo_leds(std_msgs.msg.String("all"), std_msgs.msg.String("white"))

IRsub = rospy.Subscriber("robot/irs", robobo_msgs.msg.IRs, listener)

rospy.spin()