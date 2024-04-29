# if PowerShell scripts don't work, make sure to:
# `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned`
# in a powershell running in administrator mode.
param(
    [Parameter(Mandatory=$true, Position=0, HelpMessage="Path to the .ttt scene to load in CoppeliaSim")]
    [string] $scenePath,
    [Parameter(Mandatory=$false, Position=1, HelpMessage="The TCP port to start the CoppeliaSim API at")]
    [int] $apiPort = 23000,
)

# Presumes you have the Ubuntu 22 version of CoppeliaSim as "./CoppeliaSim.tar.xz"
New-Item -Path .\tmp_dockerfiles -ItemType Directory | Out-Null
Copy-Item -Path $scenePath -Destination .\tmp_dockerfiles\to_open.ttt

docker build --tag coppelia_sim .

Remove-Item -Path .\tmp_dockerfiles -Recurse -Force

Invoke-Expression -Command "docker run -it --rm -p $($apiPort):$($apiPort) coppelia_sim -h -GzmqRemoteApi.rpcPort=$($apiPort)"
