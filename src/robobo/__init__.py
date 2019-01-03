# from base import Robobo
from __future__ import absolute_import, print_function
from .simulation import SimulationRobobo

import sys
if sys.version_info < (3,0):
    from .hardware import HardwareRobobo
else:
    print("Hardware Connection not available in python3 :(", file=sys.stderr)