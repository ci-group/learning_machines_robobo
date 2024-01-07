# if PowerShell scripts don't work, make sure to:
# `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned`
# in a powershell running in administrator mode.
param($p1)

# Presumes you have CoppeliaSim extracted to ./CoppeliaSim
.\CoppeliaSim\coppeliaSim.exe $PSBoundParameters["p1"] "-gREMOTEAPISERVERSERVICE_19999_FALSE_TRUE"
