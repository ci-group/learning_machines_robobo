#!/usr/bin/env python2

import robobo

if __name__ == "__main__":
    rob = robobo.HardwareRobobo()
    rob.connect(uri='http://192.168.1.247:11311')

    rob.set_emotion('sad')
    rob.move(20, -20, 2000, 0)
    rob.talk('Hi, my name is Robobo 2')
    rob.sleep(1)
    rob.move(-20, 20, 2000, 0)
    rob.set_emotion('happy')