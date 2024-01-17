#!/usr/bin/env bash

# Pass the scene you want to load as first argument
# Specify the port you want as second argument. Defaults to 19999 (same as default for SimulationRobobo)
# If you want to run in headless mode, specify `-h` as a third argument
${1:?"Specify the scene you want to load as a first argument"}

# Presumes you have CoppeliaSim extracted to ./CoppeliaSim
./CoppeliaSim/coppeliaSim.sh "$1" $3 "-gREMOTEAPISERVERSERVICE_${2:-19999}_FALSE_TRUE"
