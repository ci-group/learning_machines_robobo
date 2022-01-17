#!/usr/bin/env python2
from __future__ import print_function

import time
import numpy as np
import os

import robobo
import cv2
import sys
import signal
import prey

PLAY_DURATION = 200
WHEELS_SPEED_MULTIPLIER = 10
OBJECT_DISTANCE = 5


def terminate_program(signal_number, frame):
    print("Ctrl-C received, terminating program")
    sys.exit(1)


def main():
    rob = robobo.SimulationRobobo().connect(address='145.108.83.212', port=19991)
    rob.play_simulation()
    for i in range(PLAY_DURATION):
        sens_val = np.nan_to_num(np.log(np.array(rob.read_irs())), neginf=0) * -1
        if np.sum(sens_val[3:8]) >= OBJECT_DISTANCE:
            turn_right = np.random.choice((True, False))
            if turn_right:
                lw, rw = -1, 1
            else:
                lw, rw = 1, -1
            rob.move(WHEELS_SPEED_MULTIPLIER * lw, WHEELS_SPEED_MULTIPLIER * rw, 3450)
        else:
            lw, rw = 1, 1
            rob.move(WHEELS_SPEED_MULTIPLIER * lw, WHEELS_SPEED_MULTIPLIER * rw, 1000)
    rob.pause_simulation()
    time.sleep(1)

    rob.stop_world()
    time.sleep(3)

    rob.disconnect()


if __name__ == "__main__":
    main()