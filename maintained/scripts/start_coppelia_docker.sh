#!/usr/bin/env bash

# Run coppeliasim as always, but, this time, inside a container.
# Presumes you have the Ubuntu 22 version of CoppeliaSim as "./CoppeliaSim.tar.xz"

# Pass the scene you want to load as first argument
# Specify the port you want as second argument. Defaults to 23000 (same as default for SimulationRobobo and CoppeliaSim as a whole)
${1:?"Specify the scene you want to load as a first argument"}

set -xe

# This is hard to get working on both Linux and MacOS, despite both being Unix.
# https://unix.stackexchange.com/questions/30091/fix-or-alternative-for-mktemp-in-os-x
tmpDir=$(mktemp -d 2>/dev/null || mktemp -d -t 'coppelia_build')

# Copying everything over to a temp dir, and running the docker build there.
# That way, the Dockerfile alwyas knows where the scene you want to load is, and can copy it in.
cp "$1" "$tmpDir/to_open.ttt"
cp "./Dockerfile" "$tmpDir/Dockerfile"
cp "./CoppeliaSim.tar.xz" "$tmpDir/CoppeliaSim.tar.xz"

# `env --chdir`` nor `pushd`` works on MacOS, so subshell it is.
(cd "$tmpDir" && docker build --tag coppelia_sim .)

# Removing as safely as possible
rm "$tmpDir/to_open.ttt" "$tmpDir/CoppeliaSim.tar.xz" "$tmpDir/Dockerfile"
rmdir "$tmpDir"

docker run -it --rm -p "${2:-23000}:${2:-23000}" --name "coppelia_$apiPort" coppelia_sim "-GzmqRemoteApi.rpcPort=${2:-23000}"
