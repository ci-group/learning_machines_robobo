#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def callback(data: String) -> None:
    # This is just a dummy function that logs
    rospy.loginfo(f"I heard {data}")


def listen():
    # Start up the node, give it a name (which has to be unique)
    rospy.init_node("my_listener")
    rospy.loginfo("My listener was sucessfully started")

    # Start a subscriber that listens on /test/my_publisher for Strings, and will act with callback
    rospy.Subscriber("/test/my_topic", String, callback)

    # spin is just a fancy way to do an infinite loop. It simply keeps python from exiting.
    rospy.spin()


if __name__ == "__main__":
    listen()
