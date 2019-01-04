#!/usr/bin/env python2
import time
import robobo
import cv2

if __name__ == "__main__":
    robs = [
        robobo.HardwareRobobo(camera=True).connect(address="192.168.1.247"),
        robobo.SimulationRobobo().connect(address='192.168.1.204', port=19999),
    ]
    
    for i, rob in enumerate(robs):
        print('sending commands to robot {}'.format(i))
        rob.set_emotion('sad')
        rob.move(5, -5, 1000)
        rob.talk('Hi, my name is Robobo {}'.format(i))
        rob.sleep(1)
        rob.set_emotion('happy')
        image = rob.get_image_front()
        cv2.imwrite("test_pictures_{}.png".format(i),image)
        time.sleep(0.1)

    # time.sleep(5)
    for rob in robs:
        rob.move(-5, 5, 1000)
