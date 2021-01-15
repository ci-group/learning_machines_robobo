# -*- coding: utf-8 -*-
"""
Created on Fri Jan 15 09:59:33 2021

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

sol = np.loadtxt('fittestseed111mixedFull.csv')
rob = robobo.SimulationRobobo(number = '#0').connect(address='192.168.178.206', port=19997)
controller = Controller(sol)
rob.play_simulation()
start = time.time()                          
while time.time() -start < 15:
    x,y =controller.act(rob.read_irs())
    rob.move(float(x),float(y), 2000)
rob.stop_world()

