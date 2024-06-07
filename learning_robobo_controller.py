#!/usr/bin/env python3
import sys
print("THE PATH IS", sys.path)

from robobo_interface import SimulationRobobo
from learning_machines import run_all_actions, HardwareRobobo, move_till_obstacle

if __name__ == "__main__":
    # You can do better argument parsing than this!
    if len(sys.argv) < 2:
        raise ValueError(
            """To run, we need to know if we are running on hardware of simulation
            Pass `--hardware` or `--simulation` to specify."""
        )
    elif sys.argv[1] == "--hardware":
        rob = HardwareRobobo(camera=True)
    elif sys.argv[1] == "--simulation":
        rob = SimulationRobobo()
    else:
        raise ValueError(f"{sys.argv[1]} is not a valid argument.")

    # run_all_actions(rob)
    move_till_obstacle(rob)

        # plot_sensor_data(rob)