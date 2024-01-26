# if PowerShell scripts don't work, make sure to:
# `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned`
# in a powershell running in administrator mode.

# This passes trough any and all parameters to the container
param(
    [Parameter(ValueFromRemainingArguments = $true, Position = 0)]
    [String[]] $arguments
)

docker build --tag learning_machines .
# Mounting to a directory that does not exist creates it.
# Mounting to relative path doesn't create non-existing directories on Windows.
Invoke-Expression -Command "docker run -t --rm -p 45100:45100 -p 45101:45101 -v ""${PWD}/results:/root/results"" learning_machines $arguments"
