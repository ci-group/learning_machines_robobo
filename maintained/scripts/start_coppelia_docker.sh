#!/usr/bin/env bash

# Run coppeliasim as always, but, this time, inside a container.
# Presumes you have the Ubuntu 22 version of CoppeliaSim as "./CoppeliaSim.tar.xz"

# Pass the scene you want to load as first argument
# Specify the port you want as second argument. Defaults to 23000 (same as default for SimulationRobobo and CoppeliaSim as a whole)
${1:?"Specify the scene you want to load as a first argument"}

set -xe

# I am presuming this dir does not exist. It cannot go to /tmp, as Docker doesn't like non-local files.
# If you did create a dir with this name, shame on you, and please rename it, or refactor these scripts. :P
mkdir ./tmp_dockerfiles
cp $1 ./tmp_dockerfiles/to_open.ttt

docker build --tag coppelia_sim .

unlink ./tmp_dockerfiles/to_open.ttt
rmdir ./tmp_dockerfiles

docker run -it --rm -p "${2:-23000}:${2:-23000}" coppelia_sim -h "-GzmqRemoteApi.rpcPort=${2:-23000}"
