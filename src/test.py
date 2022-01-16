#!/usr/bin/env python2
from __future__ import print_function

import time
import numpy as np
import multiprocessing

import robobo
import cv2
import sys
import signal
import prey


def terminate_program(signal_number, frame):
    print("Ctrl-C received, terminating program")
    sys.exit(1)

def run_robot(port, return_dict):
    rob = robobo.SimulationRobobo().connect(address='192.168.1.101', port=port)
    rob.play_simulation()

    return_dict[port] = 10
    rob.pause_simulation()
    time.sleep(1)
    # Stopping the simualtion resets the environment
    rob.stop_world()
    time.sleep(3)
    rob.disconnect()


def main():
    manager = multiprocessing.Manager()
    return_dict = manager.dict()

    signal.signal(signal.SIGINT, terminate_program)

    p1 = multiprocessing.Process(target=run_robot, args=(19991, return_dict, ))
    p2 = multiprocessing.Process(target=run_robot, args=(19992, return_dict, ))

    p1.start()
    p2.start()

    p1.join()
    p2.join()
    print(np.average(return_dict.values()))


if __name__ == "__main__":
    main()