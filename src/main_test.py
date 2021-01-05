#!/usr/bin/env python3
from __future__ import print_function
import time
import numpy as np
import robobo
import sys
import signal
import prey


class Environment:
    ITERATIONS = 10_000
    LEARNING_RATE = .1
    DISCOUNT_FACTOR = .95

    action_space = None
    state_space = None
    observation_space = None  # All possible observations; rob's 8 sensors. Can become quite large, therefore
    # needs bucketing.

    def __init__(self):
        signal.signal(signal.SIGINT, self.terminate_program)
        self.rob = robobo.SimulationRobobo().connect(address='192.168.2.25', port=19997)
        self.q_table = self.initialize_q_table()

    def start_environment(self):
        self.rob.play_simulation()
        # Start iteration/epoch or whatever
        # Find rob's current state
        # Find best action, given this state
        # Perform action (rob.move())
        # Update Q-table

    @staticmethod
    def terminate_program():
        print("Ctrl-C received, terminating program")
        sys.exit(1)

    @staticmethod
    def initialize_q_table():
        # Initialize Q-table for states * action pairs with default values (0).
        # Since observation space is very large, we need to trim it down (bucketing) to only a select amount of
        # possible states, e.g. 4 for each sensor (4^8 = 65k). Or: use less sensors (no rear sensors for task 1).
        return "pass"


def main():
    env = Environment()
    env.start_environment()


if __name__ == "__main__":
    main()
