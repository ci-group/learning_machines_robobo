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

NUMBER_OF_GENERATIONS = 20
NUMBER_OF_INPUTS = 8
NUMBER_OF_OUTPUTS = 2
NUMBER_OF_HIDDEN_Neurons_1 = 2
NUMBER_OF_HIDDEN_Neurons_2  = 2
NUMBER_OF_WEIGHTS =(NUMBER_OF_INPUTS + 1) * NUMBER_OF_HIDDEN_Neurons_1+ (NUMBER_OF_HIDDEN_Neurons_1 + 1) *NUMBER_OF_HIDDEN_Neurons_2 +  (NUMBER_OF_HIDDEN_Neurons_2 + 1) * NUMBER_OF_OUTPUTS
NUMBER_OF_SIGMAS = NUMBER_OF_WEIGHTS

MU = 10  # how many parents per generation
LAMBDA = 30  # how many children per generation
LOWER = NUMBER_OF_WEIGHTS*[-1]
UPPER = NUMBER_OF_WEIGHTS*[1]

rob2 = robobo.SimulationRobobo(number = '#0').connect(address='192.168.178.206', port=19997)
rob = robobo.SimulationRobobo(number = '#0').connect(address='192.168.178.206', port=19998)






def fitness(c,weights):
    fitnessScore = 0
    number_of_collisions = 0
    controller = c(weights)
    while(rob.is_simulation_running()):
        pass
    rob.play_simulation()
    start = rob.get_sim_time()
    prior_irs = 'None'
    min2_irs = 'None'
    min3_irs = 'None'
    min4_irs = 'None'
    while(rob.get_sim_time() - start < 60*1000):
        current_location_x, current_location_y, current_location_z = rob.position()
        x,y =controller.act(rob.read_irs())
        rob.move(float(x),float(y), 500)
        
        if not prior_irs == 'None':
            irs = rob.read_irs()
            if 'None' not in [min2_irs, min3_irs, min4_irs]:
                rel_diff_j = 0
                rel_diff_j_minus1 = 0
                rel_diff_j_minus2 = 0
                rel_diff_j_minus3 = 0
                for i in range(NUMBER_OF_INPUTS):
                    if not (irs[i] or prior_irs[i]): #if any of the 2 dont measure anything
                        rel_diff_j+= 0.2*np.abs(irs[i]-prior_irs[i])
                    if not (min2_irs[i] or prior_irs[i]): #if any of the 2 dont measure anything
                        rel_diff_j_minus1+= 0.2*np.abs(prior_irs[i]-min2_irs[i] )
                    if not (min2_irs[i] or min3_irs[i]): #if any of the 2 dont measure anything
                        rel_diff_j_minus2+= 0.2*np.abs(min3_irs[i]-min2_irs[i] )
                    if not (min2_irs[i] or min3_irs[i]): #if any of the 2 dont measure anything
                        rel_diff_j_minus3+= 0.2*np.abs(min3_irs[i]-min4_irs[i] )
                if ((rel_diff_j_minus3 > 0.03) and (rel_diff_j_minus1 < 0.01) and rel_diff_j < 0.001):
                    number_of_collisions+=1
            min4_irs = min3_irs
            min3_irs = min2_irs
            min2_irs = prior_irs
            prior_irs = irs
                    
                        
        else: prior_irs = rob.read_irs()
        
            
        
        new_location_x, new_location_y, new_location_z = rob.position()
        fitnessScore+=np.sqrt((new_location_x - current_location_x)**2+(new_location_y - current_location_y)**2)
    
    rob.stop_world()
    
    
    
    while(rob2.is_simulation_running()):
        pass
    rob2.play_simulation()
    start = rob2.get_sim_time()
    prior_irs = 'None'
    min2_irs = 'None'
    min3_irs = 'None'
    min4_irs = 'None'
    while(rob2.get_sim_time() - start < 60*1000):
        current_location_x, current_location_y, current_location_z = rob2.position()
        x,y =controller.act(rob2.read_irs())
        rob2.move(float(x),float(y), 2000)
        
        if not prior_irs == 'None':
            irs = rob2.read_irs()
            if 'None' not in [min2_irs, min3_irs, min4_irs]:
                rel_diff_j = 0
                rel_diff_j_minus1 = 0
                rel_diff_j_minus2 = 0
                rel_diff_j_minus3 = 0
                for i in range(NUMBER_OF_INPUTS):
                    if not (irs[i] or prior_irs[i]): #if any of the 2 dont measure anything
                        rel_diff_j+= 0.2*np.abs(irs[i]-prior_irs[i])
                    if not (min2_irs[i] or prior_irs[i]): #if any of the 2 dont measure anything
                        rel_diff_j_minus1+= 0.2*np.abs(prior_irs[i]-min2_irs[i] )
                    if not (min2_irs[i] or min3_irs[i]): #if any of the 2 dont measure anything
                        rel_diff_j_minus2+= 0.2*np.abs(min3_irs[i]-min2_irs[i] )
                    if not (min2_irs[i] or min3_irs[i]): #if any of the 2 dont measure anything
                        rel_diff_j_minus3+= 0.2*np.abs(min3_irs[i]-min4_irs[i] )
                if ((rel_diff_j_minus3 > 0.03) and (rel_diff_j_minus1 < 0.01) and rel_diff_j < 0.001):
                    number_of_collisions+=1
            min4_irs = min3_irs
            min3_irs = min2_irs
            min2_irs = prior_irs
            prior_irs = irs
                    
                        
        else: prior_irs = rob2.read_irs()
        
            
        
        new_location_x, new_location_y, new_location_z = rob2.position()
        fitnessScore+=np.sqrt((new_location_x - current_location_x)**2+(new_location_y - current_location_y)**2)
    
    rob2.stop_world()

    print('working')
    return [10*fitnessScore+ 50/(number_of_collisions+1)]
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


  

pd.DataFrame(logbook).to_csv("./logbookseed{}.csv".format(seed), index=False)
pd.DataFrame(np.array(fittest)[0,]).to_csv("./fittestseed{}mixedFull.csv".format(seed), header=False, index=False)
