#!/usr/bin/env python2
from __future__ import print_function
import time
import robobo
import cv2
import sys
import signal

def terminate_program(signal_number, frame):
    print("Ctrl-C received, terminating program")
    sys.exit(1)

def main():
    signal.signal(signal.SIGINT, terminate_program)

    # rob = robobo.HardwareRobobo(camera=True).connect(address="192.168.1.5")
    rob = robobo.SimulationRobobo().connect(address='192.168.1.10', port=19997)

    rob.play_simulation()
    # rob.pause_simulation()
    
    # move and talk
    # rob.set_emotion('sad')
    
    print("robobo is at {}".format(rob.position()))
    rob.move(5, 5, 2000)
    print("robobo is at {}".format(rob.position()))

    # # Following code moves the phone stand
    # rob.set_phone_pan(343, 100)
    # rob.set_phone_tilt(109, 100)
    # time.sleep(1)
    # rob.set_phone_pan(11, 100)
    # rob.set_phone_tilt(26, 100)

    # rob.talk('Hi, my name is Robobo')
    # rob.sleep(1)
    # rob.set_emotion('happy')

    # Following code gets an image from the camera
    # image = rob.get_image_front()
    # cv2.imwrite("test_pictures.png",image)

    # time.sleep(0.1)

    # # IR reading
    # for i in range(10):
    #     print("ROB Irs: {}".format(rob.read_irs()))
    #     time.sleep(0.1)

    # pause the simulation and read the collected food
    rob.pause_simulation()
    print("Robobo collected {} food".format(rob.collected_food()))

    # Stopping the simualtion resets the environment 
    rob.stop_world()

if __name__ == "__main__":
    main()
