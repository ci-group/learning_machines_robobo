# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 21:23:01 2021

@author: Alex Korthouwer
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


seed = 111
def seed_everything(seed):
    """"
    Seed everything.
    """   
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)
    np.random.seed(seed)
seed_everything(seed)

NUMBER_OF_GENERATIONS = 100
NUMBER_OF_INPUTS = 8
NUMBER_OF_OUTPUTS = 2
NUMBER_OF_HIDDEN_Neurons_1 = 10
NUMBER_OF_HIDDEN_Neurons_2  = 10
NUMBER_OF_WEIGHTS =(NUMBER_OF_INPUTS + 1) * NUMBER_OF_HIDDEN_Neurons_1+ (NUMBER_OF_HIDDEN_Neurons_1 + 1) *NUMBER_OF_HIDDEN_Neurons_2 +  (NUMBER_OF_HIDDEN_Neurons_2 + 1) * NUMBER_OF_OUTPUTS
NUMBER_OF_SIGMAS = NUMBER_OF_WEIGHTS

MU = 10  # how many parents per generation
LAMBDA = 30  # how many children per generation
LOWER = NUMBER_OF_WEIGHTS*[-1]
UPPER = NUMBER_OF_WEIGHTS*[1]


rob = robobo.SimulationRobobo(number = '#0').connect(address='192.168.178.206', port=19997)




def fitness(c,weights):
    fitnessScore = 0
    controller = c(weights)
    while(rob.is_simulation_running()):
        pass
    rob.play_simulation()
    start = time.time()
    while(time.time() - start < 60):
        current_location_x, current_location_y, current_location_z = rob.position()
        x,y =controller.act(rob.read_irs())
        rob.move(float(x),float(y), 0)
        
        new_location_x, new_location_y, new_location_z = rob.position()
        fitnessScore+=np.abs(new_location_x - current_location_x)+np.abs(new_location_y - current_location_y)
    
    rob.stop_world()
    return [fitnessScore]
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
MIN_VALUE, MAX_VALUE = -1. , 1.
MIN_STRAT, MAX_STRAT = -1. , 1.
toolbox.register("individual", generateWeights, creator.Individual,                                  #defines an ES individual
                 creator.Strategy, NUMBER_OF_WEIGHTS, MIN_VALUE, MAX_VALUE, MIN_STRAT, 
                 MAX_STRAT)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
MIN_STRATEGY = 0.1
def checkStrategy(minstrategy):   #modifies mutation strats so it doesn't get too small
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
toolbox.register("mutate", tools.mutESLogNormal, c =1., indpb=0.3)
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


  

pd.DataFrame(logbook).to_csv("./logbookseed{}.csv".format(seed), index=False)
pd.DataFrame(np.array(fittest)[0,]).to_csv("./fittestseed{}mixedFull.csv".format(seed), header=False, index=False)