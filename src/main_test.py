#!/usr/bin/env python3
from __future__ import print_function
import time
import numpy as np
import robobo
import sys
import signal
import prey


class Environment():
    def __init__(self):
        signal.signal(signal.SIGINT, self.terminate_program)
        self.rob = robobo.SimulationRobobo().connect(address='192.168.2.25', port=19997)

    def start_environment(self):
        self.rob.play_simulation()

    @staticmethod
    def terminate_program():
        print("Ctrl-C received, terminating program")
        sys.exit(1)


def main():
    env = Environment()
    env.start_environment()


if __name__ == "__main__":
    main()
