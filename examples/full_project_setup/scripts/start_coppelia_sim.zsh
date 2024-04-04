#!/usr/bin/env zsh

#Pass the scene you want to load as first argument
# If you want to run in headless mode, specify `-h` as a second argument
${1:?"Specify the scene you want to load as a first argument"}

# Presumes you have CoppeliaSim extracted to ./coppeliaSim.app
./coppeliaSim.app/Contents/MacOS/coppeliaSim "$1" $2 
