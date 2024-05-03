# if PowerShell scripts don't work, make sure to:
# `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned`
# in a powershell running in administrator mode.

# This passes trough any and all parameters to the container
param(
    [Parameter(ValueFromRemainingArguments = $true, Position = 0)]
    [String[]] $arguments
)

# Replace newlines to `\n`, as `\r\n` is still quite unstable with Docker, even for the dockerfile.
(Get-Content "$(Get-Location)\Dockerfile") -join "`n" | Set-Content "$(Get-Location)\Dockerfile"

docker build --tag learning_machines .
# Mounting to a directory that does not exist creates it.
# Mounting to relative path doesn't create non-existing directories on Windows.
Invoke-Expression -Command "docker run -t --rm -p 45100:45100 -p 45101:45101 -v '$(Get-Location)\results:/root/results' learning_machines $arguments"
