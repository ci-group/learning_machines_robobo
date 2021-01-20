"""
Created on Mon Jan 18 2021

@author: Alex Korthouwer
@author: Yu Anlan
"""
from __future__ import unicode_literals, print_function, absolute_import, division, generators, nested_scopes
import sys, os
import numpy as np
from deap import base, creator, tools, algorithms
import pandas as pd
import random
import robobo
import cv2
import sys
import signal
import prey
import time
import array
from controller import Controller
import math
from blob_detection import detect
import calendar
from datetime import datetime

seed = 111
def seed_everything(seed):
    """"
    Seed everything.
    """
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)
    np.random.seed(seed)
seed_everything(seed)
timestamp = calendar.timegm(time.gmtime())

NUMBER_OF_GENERATIONS = 20
NUMBER_OF_INPUTS = 7
NUMBER_OF_OUTPUTS = 2
NUMBER_OF_HIDDEN_Neurons_1 = 2
NUMBER_OF_HIDDEN_Neurons_2  = 2
NUMBER_OF_WEIGHTS = (NUMBER_OF_INPUTS + 1) * NUMBER_OF_HIDDEN_Neurons_1+ (NUMBER_OF_HIDDEN_Neurons_1 + 1) \
                    * NUMBER_OF_HIDDEN_Neurons_2 +  (NUMBER_OF_HIDDEN_Neurons_2 + 1) * NUMBER_OF_OUTPUTS
NUMBER_OF_SIGMAS = NUMBER_OF_WEIGHTS

MU = 10  # how many parents per generation
LAMBDA = 30  # how many children per generation
LOWER = NUMBER_OF_WEIGHTS*[-1]
UPPER = NUMBER_OF_WEIGHTS*[1]

with open('IP.txt', 'r') as f:
    ip = f.readlines()[0]


rob = robobo.SimulationRobobo(number='').connect(address=ip, port=19997)


def fitness(c, weights):
    rob.set_phone_tilt(0.8, 50)
    fitness_score = 0
    controller = c(weights, 7, 2, 2)
    for i in range(5):
        while (rob.is_simulation_running()):
            pass
        rob.play_simulation()
        start = rob.get_sim_time()
        while (rob.get_sim_time() - start < 60 * 1000):
            sensors = rob.read_irs()[3:] # front sensors, length 5
            sensors = [sensors[i] * 5 if sensors[i] is not False else 1 for i in range(len(sensors))]
            detection_x, detection_y = detect(rob.get_image_front())

            if detection_x is False:
                detection_x, detection_y = 64, 0
            else:
                detection_x, detection_y = detection_x / 128, detection_y / 128
            inputs = sensors + [detection_x, detection_y]
            x, y = controller.act(inputs)
            rob.move(float(x), float(y), 500)

            fitness_score += rob.collected_food()

        rob.stop_world()

    print('working')
    print(fitness_score)
    return [fitness_score]


# initialize fitness and set fitness weight to positive value (we want to maximize)
creator.create("FitnessMax", base.Fitness, weights=[1.0])
# the goal ('fitness') function to be maximized

creator.create("Individual", array.array, typecode="d",
               fitness=creator.FitnessMax, strategy=None)
creator.create("Strategy", array.array, typecode="d")
record = 0


def generateWeights(icls, scls, size, imin, imax, smin, smax):
    ind = icls(np.random.normal() for _ in range(size))
    ind.strategy = scls(random.gauss(0, 1) for _ in range(size))

    return ind


toolbox = base.Toolbox()

# generation functions
MIN_VALUE, MAX_VALUE = -1., 1.
MIN_STRAT, MAX_STRAT = -1., 1.
toolbox.register("individual", generateWeights, creator.Individual,  # defines an ES individual
                 creator.Strategy, NUMBER_OF_WEIGHTS, MIN_VALUE, MAX_VALUE, MIN_STRAT,
                 MAX_STRAT)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
MIN_STRATEGY = 0.1


def checkStrategy(minstrategy):  # modifies mutation strats so it doesn't get too small
    def decorator(func):
        def wrappper(*args, **kargs):
            children = func(*args, **kargs)
            for child in children:
                for i, s in enumerate(child.strategy):
                    if s < minstrategy:
                        child.strategy[i] = minstrategy
            return children

        return wrappper

    return decorator


toolbox.register("evaluate", fitness, Controller)
toolbox.register("mate", tools.cxBlend, alpha=0.1)
toolbox.register("mutate", tools.mutESLogNormal, c=1.0, indpb=0.3)
toolbox.register("select", tools.selTournament, tournsize=3)

stats = tools.Statistics(lambda ind: ind.fitness.values)
stats.register("avg", np.mean)
stats.register("std", np.std)
stats.register("min", np.min)
stats.register("max", np.max)
fittest = tools.HallOfFame(10)

population = toolbox.population(n=MU)

population, logbook = algorithms.eaMuCommaLambda(population, toolbox, mu=MU, lambda_=LAMBDA,
                                                 cxpb=0.2, mutpb=0.7, ngen=NUMBER_OF_GENERATIONS, stats=stats,
                                                 halloffame=fittest, verbose=True)

pd.DataFrame(logbook).to_csv(
    "./generations_fitness_{}_{}.csv".format(seed, timestamp), index=False
)
pd.DataFrame(np.array(fittest)[0,]).to_csv(
    "./weights_{}_{}.csv".format(seed, timestamp), header=False, index=False
)

