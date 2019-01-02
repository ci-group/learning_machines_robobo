#!/usr/bin/env python2

import rospy
from robobo_msgs.msg import IRs

print("hello world")

rospy.init_node("robobo__demo")
print("init_node")

def listener(info):
    print('Info: ', info)

sub=rospy.Subscriber("/robot/irs", IRs, listener)
print("subscribe irs")

rospy.spin()
print("spin")