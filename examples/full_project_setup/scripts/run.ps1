# if PowerShell scripts don't work, make sure to:
# `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned`
# in a powershell running in administrator mode.
param($p1)

docker build --tag learning_machines .
docker run -t -p 45100:45100 -p 45101:45101 learning_machines $PSBoundParameters["p1"]