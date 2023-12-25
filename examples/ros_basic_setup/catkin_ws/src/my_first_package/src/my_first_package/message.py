import numpy


# This is the message we send
# The reason this is seperated out is to show you how to get code from packages.
def get_message() -> str:
    return "Hello ROS"


# This is another random function to show the dependencies work.
def get_random_value() -> float:
    return numpy.random.randint(1, 100)
