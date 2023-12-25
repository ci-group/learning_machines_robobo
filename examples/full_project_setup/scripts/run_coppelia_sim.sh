#!/bin/bash

start_port=19999

xvfb-run ./coppeliaSim.sh -h scenes/robobo_food_arena.ttt -gREMOTEAPISERVERSERVICE_${start_port}_FALSE_TRUE
