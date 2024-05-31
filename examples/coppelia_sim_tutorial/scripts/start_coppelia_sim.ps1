# if PowerShell scripts don't work, make sure to:
# `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned`
# in a powershell running in administrator mode.
param(
    [Parameter(Mandatory=$true, Position=0, HelpMessage="Path to the .ttt scene to load in CoppeliaSim")]
    [string] $scenePath,
    [Parameter(Mandatory=$false, Position=1, HelpMessage="The TCP port to start the CoppeliaSim API at")]
    [int] $apiPort = 23000,
    [Parameter(Mandatory=$false, Position=2, HelpMessage="Wether to run in headless (GUI-less) mode")]
    [switch] $headless
)

$h = if ($headless.IsPresent) {"-h"} else {""}

# Presumes you have CoppeliaSim extracted to ./CoppeliaSim
.\CoppeliaSim\coppeliaSim.exe "$scenePath" $h "-GzmqRemoteApi.rpcPort=$($apiPort)"
