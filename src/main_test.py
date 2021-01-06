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


class Direction:
    LEFT = (-25, 25, 300)  # Action: 0
    RIGHT = (25, -25, 300)  # Action: 1
    FORWARD = (25, 25, 300)  # Action: 2
    RRIGHT = (50, -50, 300)  # Action: 3
    LLEFT = (-50, 50, 300)  # Action: 4


class Environment:
    MAX_ITERATIONS = 10_000
    MAX_SIMULATION_ITERATIONS = 500
    LEARNING_RATE = .1
    DISCOUNT_FACTOR = .95
    EPSILON_LOW = .6
    EPSILON_HIGH = .99

    action_space = [0, 1, 2, 3, 4]
    collision_boundary = .1
    collision_counter, iteration_counter, epsilon_counter = 0, 0, 0
    epsilon_increase = int((MAX_ITERATIONS // (EPSILON_HIGH - EPSILON_LOW) * 100) / 10_000)

    def __init__(self):
        signal.signal(signal.SIGINT, self.terminate_program)
        self.rob = robobo.SimulationRobobo().connect(address='192.168.2.25', port=19997)
        self.q_table = self.initialize_q_table()
        self.discrete_os_win_size = []

    def start_environment(self):
        for i in range(1, self.MAX_ITERATIONS):
            print(f"Starting simulation nr. {i}")

            self.rob.play_simulation()
            while self.valid_environment():
                curr_state = self.handle_state()
                if random.random() < (1 - self.EPSILON_LOW):
                    best_action = random.choice(self.action_space)
                else:
                    best_action = np.argmax(self.q_table[curr_state])

                self.update_q_table(best_action, curr_state)
                self.change_epsilon()
                self.iteration_counter += 1
            else:
                print(f"Environment is not valid anymore, starting new environment")
                self.iteration_counter = 0
                self.rob.stop_world()

    @staticmethod
    def terminate_program():
        print("Ctrl-C received, terminating program")
        sys.exit(1)

    def valid_environment(self):
        if self.collision_counter > 100:
            return False
        if self.iteration_counter >= self.MAX_SIMULATION_ITERATIONS:
            return False

        return True

    def change_epsilon(self):
        if self.epsilon_counter == self.epsilon_increase:
            if self.EPSILON_LOW < self.EPSILON_HIGH:
                self.EPSILON_LOW += 0.01
                print(f"Increasing epsilon to {self.EPSILON_LOW}")
                self.epsilon_counter = 0
        else:
            self.epsilon_counter += 1

    def initialize_q_table(self):
        # Initialize Q-table for states * action pairs with default values (0).
        # Since observation space is very large, we need to trim it down (bucketing) to only a select amount of
        # possible states, e.g. 4 for each sensor (4^8 = 65k). Or: use less sensors (no rear sensors for task 1).
        q_table = np.random.uniform(low=0, high=0, size=([5, 5, 5, 5, 5] + [len(self.action_space)]))
        q_table[(0, 0, 0, 0, 0)][2] = 1  # Initialize the table with forward move.
        return np.round(q_table)

    def handle_state(self):
        # This function should return the values with which we can index our q_table, in tuple format.
        # So it should take the last 5 sensor inputs (current state), transform each of them into a bucket where
        # the bucket size is already determined by the shape of the q_table.
        sensor_values = np.log(np.array(self.rob.read_irs())[3:]) / 10
        sensor_values = np.where(sensor_values == -np.inf, 0, sensor_values)  # Remove the infinite values.
        sensor_values = (sensor_values - -0.65) / 0.65  # Scales all variables between [0, 1] where 0 is close proximity.

        # Check what the actual sensor_values are (between [0, 1]) and determine their state
        indices = []
        for sensor_value in sensor_values:
            if 1 >= sensor_value >= 0.8:  # No need for action, moving forward is best.
                indices.append(0)
            elif 0.8 > sensor_value >= 0.6:  # No need for action, moving forward is best.
                indices.append(1)
            elif 0.6 > sensor_value >= 0.4:  # No need for action, moving forward is best.
                indices.append(2)
            elif 0.4 > sensor_value >= 0.2:  # No need for action, moving forward is best.
                indices.append(3)
            elif 0.2 > sensor_value >= 0:  # No need for action, moving forward is best.
                indices.append(4)

        return tuple(indices)

    def handle_action(self, action):
        # This function should accept an action (0, 1, 2...) and move the robot accordingly (left, right, forward).
        # Return: new_state and reward
        reward = -1

        # Keep track of collision. If it reaches its threshold, reset environment.
        collision = self.collision()

        if action == 0:
            left, right, duration = Direction.LEFT  # Left, action 0
            # self.rob.move(-10, -10, 50)
            # if collision:
            #     reward += 1
            # else:
            #     reward -= 1
        elif action == 1:
            left, right, duration = Direction.RIGHT  # Right, action 1
            # self.rob.move(-10, -10, 50)
            # if collision:
            #     reward += 1
            # else:
            #     reward -= 1
        elif action == 3:
            left, right, duration = Direction.RRIGHT  # Extreme right, action 3
            # self.rob.move(-10, -10, 50)
            # if collision:
            #     reward += 1
            # else:
            #     reward -= 1
        elif action == 4:
            left, right, duration = Direction.LLEFT  # Extreme left, action 4
            # self.rob.move(-10, -10, 50)
            # if collision:
            #     reward += 1
            # else:
            #     reward -= 1
        else:
            left, right, duration = Direction.FORWARD  # Forward, action 2
            if collision:
                reward -= 4
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
            return True
        else:
            self.collision_counter = 0
            return False

    def update_q_table(self, best_action, curr_state):
        new_state, reward = self.handle_action(best_action)
        max_future_q = np.amax(self.q_table[new_state])
        current_q = self.q_table[curr_state][best_action]

        # Calculate the new Q-value with the common formula
        new_q = (1 - self.LEARNING_RATE) * current_q + self.LEARNING_RATE * (
                reward + self.DISCOUNT_FACTOR * max_future_q)

        self.q_table[curr_state][best_action] = new_q  # And update the value


def main():
    env = Environment()
    env.start_environment()


if __name__ == "__main__":
    main()
