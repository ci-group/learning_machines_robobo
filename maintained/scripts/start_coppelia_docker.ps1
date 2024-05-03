# if PowerShell scripts don't work, make sure to:
# `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned`
# in a powershell running in administrator mode.
param(
    [Parameter(Mandatory = $true, Position = 0, HelpMessage = "Path to the .ttt scene to load in CoppeliaSim")]
    [string] $scenePath,
    [Parameter(Mandatory = $false, Position = 1, HelpMessage = "The TCP port to start the CoppeliaSim API at")]
    [int] $apiPort = 23000
)

# For some reason, "Path.GetTempFileName" exists, creating a new temp file,
# but there is no "Path.GetTempDirName", so we have to do it ourselves like this.
$tempDir = [System.IO.Path]::Combine([System.IO.Path]::GetTempPath(), [System.IO.Path]::GetRandomFileName())
New-Item -Path "$tempDir" -ItemType Directory | Out-Null

# Copying everything over to a temp dir, and running the docker build there.
# That way, the Dockerfile alwyas knows where the scene you want to load is, and can copy it in.
Copy-Item -Path "$scenePath" -Destination (Join-Path -Path "$tempDir" -ChildPath "to_open.ttt")
Copy-Item -Path "$(Get-Location)\CoppeliaSim.tar.xz" -Destination "$tempDir"
# Replace newlines to `\n` during copy, as `\r\n` is still quite unstable with Docker, even for the dockerfile.
(Get-Content "$(Get-Location)\Dockerfile") -join "`n" | Set-Content "$($tempDir)\Dockerfile"

Push-Location -Path "$tempDir"
docker build -t coppelia_sim .
Pop-Location
Remove-Item -Path "$tempDir" -Recurse -Force

Invoke-Expression -Command "docker run -it --rm -p '$($apiPort):$($apiPort)' --name 'coppelia_$($apiPort)' coppelia_sim '-GzmqRemoteApi.rpcPort=$($apiPort)'"
