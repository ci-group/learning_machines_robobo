# if PowerShell scripts don't work, make sure to:
# `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned`
# in a powershell running in administrator mode.
#
# Sadly, there is no good equivilant to "$@" in ps1
param($p1)

docker build --tag learning_machines .
# Mounting to a directory that does not exist creates it.
# Mounting to relative paths works since docker engine 23
docker run -t --rm -p 45100:45100 -p 45101:45101 -v ./results:/root/results learning_machines $PSBoundParameters["p1"]
