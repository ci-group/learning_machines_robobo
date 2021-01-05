#!/usr/bin/env python3
from __future__ import print_function
import time
import numpy as np
import robobo
import sys
import signal
import prey
import random
import pprint


class Environment:
    ITERATIONS = 10_000
    DURATION_SIMULATION = 250
    LEARNING_RATE = .1
    DISCOUNT_FACTOR = .95
    EPSILON = .8

    action_space = [0, 1, 2]
    state_space = None
    observation_space = None  # All possible observations; rob's 8 sensors. Can become quite large, therefore
    # needs bucketing.
    collision_boundary = .05
    collision_counter = 0

    def __init__(self):
        signal.signal(signal.SIGINT, self.terminate_program)
        self.rob = robobo.SimulationRobobo().connect(address='192.168.2.25', port=19997)
        self.q_table = self.initialize_q_table()
        self.discrete_os_win_size = []

    def start_environment(self):
        for it in range(self.ITERATIONS):
            self.rob.play_simulation()

            while self.valid_environment():
                curr_state = self.handle_state()
                if random.random() < (1 - self.EPSILON):
                    best_action = random.choice(self.action_space)
                else:
                    best_action = np.argmax(self.q_table[curr_state])

                self.update_q_table(best_action)
            else:
                self.rob.stop_world()

    @staticmethod
    def terminate_program():
        print("Ctrl-C received, terminating program")
        sys.exit(1)

    def valid_environment(self):
        if self.collision_counter > 100:
            return False
        return True

    def initialize_q_table(self):
        # Initialize Q-table for states * action pairs with default values (0).
        # Since observation space is very large, we need to trim it down (bucketing) to only a select amount of
        # possible states, e.g. 4 for each sensor (4^8 = 65k). Or: use less sensors (no rear sensors for task 1).
        q_table = np.random.uniform(low=0, high=2, size=([2, 2, 2, 2, 2] + [len(self.action_space)]))
        q_table[(0, 0, 0, 0, 0)] = np.array([0, 0, 1])  # Initialize the table with forward move
        return np.round(q_table)

    def handle_state(self):
        # This function should return the values with which we can index our q_table, in tuple format.
        # So it should take the last 5 sensor inputs (current state), transform each of them into a bucket where
        # the bucket size is already determined by the shape of the q_table.
        sensor_values = np.log(np.array(self.rob.read_irs())[3:]) / 10
        sensor_values = np.where(sensor_values == -np.inf, 0, sensor_values)  # Remove the infinite values.
        sensor_values = (sensor_values - -0.65) / 0.65  # Scales all variables between [0, 1] where 0 is close.

        # Check what the actual sensor_values are (between [0, 1]) and determine their state
        indices = []
        for sensor_value in sensor_values:
            if 1 >= sensor_value >= 0.7:  # No need for action, moving forward is best.
                indices.append(0)
            else:
                indices.append(1)  # Action!

        return tuple(indices)

    def handle_action(self, action):
        # This function should accept an action (0, 1, 2) and move the robot accordingly (left, right, forward).
        # Return: new_state and reward
        reward = 0

        # Keep track of collision. If it reaches its threshold, reset environment.
        collision = self.collision()

        if action == 0:
            left, right, duration = -40, 40, 200  # Left
            if collision:
                reward += 1
            else:
                reward -= 1
        elif action == 1:
            left, right, duration = 40, -40, 200  # Right
            if collision:
                reward += 1
            else:
                reward -= 1
        else:
            left, right, duration = 40, 40, 200  # Forward
            if collision:
                reward -= 1
            else:
                reward += 2

        self.rob.move(left, right, duration)

        return self.handle_state(), reward  # New_state, reward

    def collision(self):
        # If one of the (absolute) sensor values < threshold, return True.
        sensor_values = np.array(self.rob.read_irs())[3:]
        collision = any([0 < i < self.collision_boundary for i in sensor_values])

        if collision:
            self.collision_counter += 1
            print(f"Collision counter is now: {self.collision_counter}")
            return True
        else:
            self.collision_counter = 0
            return False

    def update_q_table(self, best_action):
        new_state, reward = self.handle_action(best_action)
        max_future_q = np.amax(self.q_table[new_state])
        current_q = self.q_table[new_state][best_action]

        # Calculate the new Q-value with the common formula
        new_q = (1 - self.LEARNING_RATE) * current_q + self.LEARNING_RATE * (
                reward + self.DISCOUNT_FACTOR * max_future_q)

        print(f"Current Q: {current_q}, received reward: {reward}, new Q: {new_q}")

        self.q_table[new_state][best_action] = new_q  # And update the value


def main():
    env = Environment()
    env.start_environment()


if __name__ == "__main__":
    main()
