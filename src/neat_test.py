#!/usr/bin/env python2
from __future__ import print_function

import time
import numpy as np
import os
import neat
import pickle

import robobo
import cv2
import sys
import signal
import prey


def terminate_program(signal_number, frame):
    print("Ctrl-C received, terminating program")
    sys.exit(1)


def eval_genomes(genomes, config):
    for genome_id, genome in genomes:
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        genome.fitness = 0

        signal.signal(signal.SIGINT, terminate_program)
        rob = robobo.SimulationRobobo().connect(address='192.168.1.101', port=19997)
        rob.play_simulation()
        for i in range(200):
            sens_val = np.nan_to_num(np.log(np.array(rob.read_irs())), neginf=0) * -1
            #There is no obstacle
            if np.sum(sens_val) == 0:
                lw, rw = net.activate(sens_val)
                if lw > 0 and rw > 0:
                    genome.fitness += abs(lw + rw)
                else:
                    genome.fitness += 0.5 * abs(lw + rw)
            #There is an obstacle
            else:
                norm_sens_val = (sens_val - np.min(sens_val)) / (np.max(sens_val) - np.min(sens_val))
                lw, rw = net.activate(norm_sens_val)
                dist_obj = np.sum(sens_val) / 15
                if lw > 0 and rw > 0:
                    genome.fitness += abs(lw + rw) * (1 - dist_obj)
                else:
                    genome.fitness += 0.5 * abs(lw + rw) * (1 - dist_obj)

            rob.move(10*lw, 10*rw, 1000)
        print("Player:", genome_id, "fitness-", genome.fitness)
        # pause the simulation and read the collected food
        rob.pause_simulation()
        time.sleep(2)
        # Stopping the simualtion resets the environment
        rob.stop_world()
        time.sleep(5)
        rob.disconnect()

def run(config_file):
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_file)

    pop = neat.Population(config)
    pop.add_reporter(neat.StdOutReporter(True))
    pop.add_reporter(neat.StatisticsReporter())
    pop.add_reporter(neat.Checkpointer(5, None, "saves/save-test"))

    winner = pop.run(eval_genomes, 50)
    with open(f'best', 'wb') as f:
        pickle.dump(winner, f)

if __name__ == "__main__":
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, "config-feedforward.txt")
    run(config_path)
    # main()