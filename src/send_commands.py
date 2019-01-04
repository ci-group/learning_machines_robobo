#!/usr/bin/env python2
from __future__ import print_function
import time
import robobo
import cv2

if __name__ == "__main__":
    robs = [
        robobo.HardwareRobobo(camera=True).connect(address="192.168.1.66"),
        robobo.SimulationRobobo().connect(address='192.168.1.204', port=19999),
    ]
    
    # move and talk
    for i, rob in enumerate(robs):
        print('sending commands to robot {}'.format(i))
        rob.set_emotion('sad')
        rob.move(5, -5, 1000)
        rob.set_phone_pan(343, 100)
        rob.set_phone_tilt(109, 100)
        time.sleep(1)
        rob.set_phone_pan(11, 100)
        rob.set_phone_tilt(26, 100)
        rob.talk('Hi, my name is Robobo {}'.format(i))
        rob.sleep(1)
        rob.set_emotion('happy')
        image = rob.get_image_front()
        cv2.imwrite("test_pictures_{}.png".format(i),image)
        time.sleep(0.1)

    # IR reading
    for i in range(10):
        for i, rob in enumerate(robs):
            print("ROB {} Irs: {}".format(i, rob.read_irs()))
        time.sleep(1)

    # move back
    for rob in robs:
        rob.move(-5, 5, 1000)
