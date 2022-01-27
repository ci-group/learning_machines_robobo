# from base import Robobo
from __future__ import absolute_import, print_function
from .simulation import SimulationRobobo
from .simulation_prey import SimulationRoboboPrey

try:
    # Tries to import Hardware, but if rospy is not installed, 
    # you are probably running only the simulator so
    # we are just firing a warning.
    from .hardware import HardwareRobobo
except ModuleNotFoundError as e:
    if e.name == 'rospy':
        print(f"WARNING! Was not able to load the hardware module, '{e.name}' module is missing")
    else:
        raise e
