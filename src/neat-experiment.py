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

PLAY_DURATION = 20
PORTS = [19991, 19992]
BACKWARDS_PENALTY = 0.7
WHEELS_SPEED_MULTIPLIER = 10
OBJECT_DISTANCE_DIVIDER = 15


def terminate_program(signal_number, frame):
    print("Ctrl-C received, terminating program")
    sys.exit(1)


def run_robot(network, port, return_dict):
    signal.signal(signal.SIGINT, terminate_program)
    rob = robobo.SimulationRobobo().connect(address='192.168.1.101', port=port)
    fitness = 0
    rob.play_simulation()

    for i in range(PLAY_DURATION):
        sens_val = np.nan_to_num(np.log(np.array(rob.read_irs())), neginf=0) * -1
        # There is no obstacle
        if np.sum(sens_val) == 0:
            lw, rw = network.activate(sens_val)
            if lw > 0 and rw > 0:
                fitness += abs(lw + rw)
            else:
                fitness += BACKWARDS_PENALTY * abs(lw + rw)
        # There is an obstacle
        else:
            norm_sens_val = (sens_val - np.min(sens_val)) / (np.max(sens_val) - np.min(sens_val))
            lw, rw = network.activate(norm_sens_val)
            dist_obj = np.sum(sens_val) / OBJECT_DISTANCE_DIVIDER

            if lw > 0 and rw > 0:
                fitness += abs(lw + rw) * (1 - dist_obj)
            else:
                fitness += BACKWARDS_PENALTY * abs(lw + rw) * (1 - dist_obj)

        rob.move(WHEELS_SPEED_MULTIPLIER * lw, WHEELS_SPEED_MULTIPLIER * rw, 1000)

    return_dict[port] = fitness

    rob.pause_simulation()
    time.sleep(1)

    rob.stop_world()
    time.sleep(3)

    rob.disconnect()


def eval_genomes(genomes, config):
    manager = multiprocessing.Manager()
    return_dict = manager.dict()

    for genome_id, genome in genomes:
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        genome.fitness = 0
        processes = []
        for i in range(2):
            processes.append(multiprocessing.Process(target=run_robot, args=(net, PORTS[i], return_dict,)))
            processes[i].start()

        for i in range(2):
            processes[i].join()

        genome.fitness = np.average(return_dict.values())
        print("Player:", genome_id, "fitness-", genome.fitness)


def run(config_file):
    gen_nr = 50
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_file)

    pop = neat.Population(config)
    pop.add_reporter(neat.StdOutReporter(True))
    pop.add_reporter(neat.StatisticsReporter())
    pop.add_reporter(neat.Checkpointer(5, None, "saves/save-test"))

    winner = pop.run(eval_genomes, gen_nr)
    with open(f'best', 'wb') as f:
        pickle.dump(winner, f)


if __name__ == "__main__":
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, "config-feedforward.txt")
    run(config_path)