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

    #TODO proper names
    def move(self, a, b, c, d):
        raise NotImplementedError("Not implemented")
    
    def talk(self, message):
        raise NotImplementedError("Not implemented")

    def set_led(self, selector, color):
        raise NotImplementedError("Not implemented")
    
    def set_irs_listener(self, listener):
        raise NotImplementedError("Not implemented")

    def sleep(self, seconds):
        time.sleep(seconds)
