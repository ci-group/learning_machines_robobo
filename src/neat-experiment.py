#!/usr/bin/env python2
from __future__ import print_function

import time
import numpy as np
import os
import neat
import pickle
import multiprocessing

import robobo
import cv2
import sys
import signal
import prey

PLAY_DURATION = 200
PORTS = [19991, 19992, 19993, 19994, 19995]
BACKWARDS_PENALTY = 0.6
WHEELS_SPEED_MULTIPLIER = 10
OBJECT_DISTANCE_DIVIDER = 15


def terminate_program(signal_number, frame):
    print("Ctrl-C received, terminating program")
    sys.exit(1)


def eval_genomes(genomes, config):
    intervals = np.arange(-1, 1.05, 0.05)
    for genome_id, genome in genomes:
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        genome.fitness = 0
        signal.signal(signal.SIGINT, terminate_program)
        rob = robobo.SimulationRobobo().connect(address='145.108.83.212', port=PORTS[np.random.randint(0, 5)])
        rob.play_simulation()

        for i in range(PLAY_DURATION):
            sens_val = np.nan_to_num(np.log(np.array(rob.read_irs())), neginf=0) * -1
            # There is no obstacle
            if np.sum(sens_val) == 0:
                lw_raw, rw_raw = net.activate(sens_val)
                lw = round(min(intervals, key=lambda x: abs(x - lw_raw)), 2)
                rw = round(min(intervals, key=lambda x: abs(x - rw_raw)), 2)

                if lw > 0 and rw > 0:
                    genome.fitness += abs(lw + rw)
                else:
                    genome.fitness += BACKWARDS_PENALTY * abs(lw + rw)
            # There is an obstacle
            else:
                lw_raw, rw_raw = net.activate(sens_val / 5)
                lw = round(min(intervals, key=lambda x: abs(x - lw_raw)), 2)
                rw = round(min(intervals, key=lambda x: abs(x - rw_raw)), 2)

                dist_obj = np.sum(sens_val) / OBJECT_DISTANCE_DIVIDER

                if lw > 0 and rw > 0:
                    genome.fitness += abs(lw + rw) * (1 - dist_obj)
                else:
                    genome.fitness += BACKWARDS_PENALTY * abs(lw + rw) * (1 - dist_obj)

            rob.move(WHEELS_SPEED_MULTIPLIER * lw, WHEELS_SPEED_MULTIPLIER * rw, 1000)

        print("Player:", genome_id, "fitness-", genome.fitness)
        rob.pause_simulation()
        time.sleep(1)

        rob.stop_world()
        time.sleep(3)

        rob.disconnect()


def run(config_file):
    gen_nr = 25
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_file)

    pop = neat.Population(config)
    pop.add_reporter(neat.StdOutReporter(True))
    pop.add_reporter(neat.StatisticsReporter())
    pop.add_reporter(neat.Checkpointer(5, None, "saves/exp1"))

    winner = pop.run(eval_genomes, gen_nr)
    with open(f'best_exp1', 'wb') as f:
        pickle.dump(winner, f)


if __name__ == "__main__":
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, "config-feedforward.txt")
    run(config_path)