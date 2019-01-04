from __future__ import unicode_literals, print_function, absolute_import, division, generators, nested_scopes
import time

class Robobo:
    def __init__(self):
        pass

    def connect(self, uri=None):
        raise NotImplementedError("Not implemented")

    def spin(self):
        raise NotImplementedError("Not implemented")

    def set_emotion(self, emotion):
        raise NotImplementedError("Not implemented")

    def move(self, left, right, millis):
        raise NotImplementedError("Not implemented")
    
    def talk(self, message):
        raise NotImplementedError("Not implemented")

    def set_led(self, selector, color):
        raise NotImplementedError("Not implemented")
    
    def read_irs(self):
        raise NotImplementedError("Not implemented")

    def get_image_front(self):
        raise NotImplementedError("Not implemented")

    def set_phone_orientation(self, a, b, c, d, e, f):
        raise NotImplementedError("Not implemented")

    def set_phone_pan(self, pan_position, pan_speed):
        """
        Command the robot to move the smartphone holder in the horizontal (pan) axis.

        Arguments

        pan_position: Angle to position the pan at.
        pan_speed: Movement speed for the pan mechanism.
        """
        raise NotImplementedError("Not implemented")

    def set_phone_tilt(self, tilt_position, tilt_speed):
        """
        Command the robot to move the smartphone holder in the vertical (tilt) axis.

        Arguments

        tilt_position: Angle to position the tilt at.
        tilt_speed: Movement speed for the tilt mechanism.
        """
        raise NotImplementedError("Not implemented")

    def sleep(self, seconds):
        time.sleep(seconds)
