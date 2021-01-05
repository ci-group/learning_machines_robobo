#!/usr/bin/env python3
from __future__ import print_function

import time
import numpy as np

import robobo
import cv2
import sys
import signal
import prey


def terminate_program(signal_number, frame):
    print("Ctrl-C received, terminating program")
    sys.exit(1)


def get_state(rob):
    sensors = np.log(np.array(rob.read_irs())) / 10
    sensors = np.where(sensors == -np.inf, 0, sensors)  # remove the infinite
    sensors = (sensors - -0.65) / 0.65

    n_sensors = []
    for sensor in sensors:
        if sensor < 0.6:
            r = 1
        else:
            r = 0
        n_sensors.append(r)

    r = max(n_sensors[3:6])
    # c = n_sensors[5]
    l = max(n_sensors[6:8])

    return int(str(l) + str(r), 2), sensors


def main():
    signal.signal(signal.SIGINT, terminate_program)

    # rob = robobo.HardwareRobobo(camera=True).connect(address="192.168.1.7")
    rob = robobo.SimulationRobobo().connect(address='192.168.2.25', port=19997)

    rob.play_simulation()

    # Following code moves the robot
    for i in range(100):
        rob.move(2, 2, 1_000)
        print(get_state(rob))

    rob.sleep(1)

    # Following code moves the phone stand
    # rob.set_phone_pan(343, 100)
    # rob.set_phone_tilt(109, 100)
    # time.sleep(1)
    # rob.set_phone_pan(11, 100)
    # rob.set_phone_tilt(26, 100)

    # Following code makes the robot talk and be emotional
    # rob.set_emotion('happy')
    # rob.talk('Hi, my name is Robobo')
    # rob.sleep(1)
    # rob.set_emotion('sad')

    # Following code gets an image from the camera
    # image = rob.get_image_front()
    # cv2.imwrite("test_pictures.png",image)

    time.sleep(0.1)

    # IR reading
    # for i in range(1000000):
    #     sensors = np.log(np.array(rob.read_irs()))/10
    #     print("ROB Irs: {}".format(sensors))
    #
    #     sensors = (sensors - -0.65) / 0.65
    #     print("ROB Irs TWO: {}".format(sensors))
    #
    #     print("ROB Irs THREE: {} \n\n".format(rob.read_irs()))
    #
    #     # [backR, backC, backL, frontRR, frontR, frontC, frontL, frontLL]
    #     # print('get_state: ', get_state(rob))
    #     time.sleep(0.1)

    # pause the simulation and read the collected food
    rob.pause_simulation()
    
    # Stopping the simualtion resets the environment
    rob.stop_world()


if __name__ == "__main__":
    main()
