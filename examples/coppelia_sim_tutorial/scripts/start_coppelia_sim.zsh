#!/bin/bash

#Pass the scene you want to load as first argument

# Presumes you have CoppeliaSim extracted to ./coppeliaSim.app
./coppeliaSim.app/Contents/MacOS/coppeliaSim $1 "-gREMOTEAPISERVERSERVICE_19999_FALSE_TRUE"
