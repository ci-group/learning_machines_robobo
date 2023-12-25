#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

# Import our own code from src/
from my_first_package import get_message, get_random_value


def publish():
    # Start up the node, give it a name (which has to be unique)
    rospy.init_node("my_publisher")
    rospy.loginfo("My publisher was succesfully started")

    # Start up a publisher connect it to a channel, specify what datatype it sends.
    pub = rospy.Publisher("/test/my_topic", String, queue_size=10)

    # This is only for rate.sleep() Specify it in herz, and the sleep will fill *until* that time.
    # Making the timing quite accurate.
    rate = rospy.Rate(10)

    # Run untill shutdown with <C-c>
    while not rospy.is_shutdown():
        # Get the message from the package we have, this is in src/
        message_str = get_message()

        # publish the message
        pub.publish(f"[{rospy.get_time()}] {message_str} {get_random_value()}")

        # You can also log it here, if you want to debug
        # rospy.loginfo(message_str)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish()
    except rospy.ROSInterruptException:
        # Catching the ROSInterruptException is recommended by the ROS docs.
        pass
