# Full project example

This is the example you are expected to use as a project template. The structure is the same as [basic_ros_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_basic_setup), it contains the same dockerfile and some of the same scripts and packages, but it contains a lot more stuff on top of that. First of all, it contains the same `start_coppelia_sim` script as [coppelia_sim_tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/coppelia_sim_tutorial), meaning you should also copy-past CoppeliaSim to here, such that `./CoppeliaSim` exists. It also contains the `setup.bash` from [hardware_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/hardware_setup) that you should configure to have the `ROS_MASTER_URI`.

### Running the code that is there

As you find it, this example is a fully working system. Currently, it contains a script that simply connects to the robot, be it hardware or software, and then runs through the motions it supports: moving itself and the phone around, getting input from the camera and the IR sensors, and a whole bunch more.

#### Running with hardware

To run it with hardware, you have to set everything up in the same way as the [hardware_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/hardware_setup), which is to say you have to confirm you're on the same (non-public) network with the phone and your laptop/desktop, and have to put the `ROS_MASTER_URI` (`http://[Adress shown on top left]:11311`) in `setup.bash`.

After this, you can run with the flag `--hardware`. If you did everything correctly, you'll then see the robot move around and do a bunch of stuff.

```sh
bash ./scripts/run.sh --hardware
```

```ps1
.\scripts\run.ps1 --hardware
```

#### Running with simulation

To run it with the simulation, you have to, first of all, make sure that CoppeliaSim is installed under `./CoppeliaSim` like in the [coppelia_sim_tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/coppelia_sim_tutorial). After that, you have to update `setup.bash` again, this time updating `COPPELIA_SIM_IP` with the IP of the computer you're running CoppeliaSim on, which is to say your own. We have to do this because the container (again, think of it as a small separate computer), has to connect to your computer to find it. Technically, though, this also means you can run the simulation on another system as this package.

To get your own IP address, you have to run `Get-NetIPAddress` on Windows PowerShell, but this gives you a bunch of extra stuff. To get _only_ your IP address, you can run:

```ps1
(Get-NetIPAddress | Where-Object { $_.AddressState -eq "Preferred" -and $_.ValidLifetime -lt "24:00:00" }).IPAddress
```

similarly, on Unix, you can run `hostname -I` to get all the information, but the following to get only the IP you are looking for:

```sh
hostname -I | awk '{print $1}'
```

After you have updated `setup.bash` you can start CoppeliaSim with the `start_coppelia_sim.sh` script like in that example:

```sh
bash ./scripts/start_coppelia_sim.sh ./scenes/Robobo_Scene.ttt
```

```ps1
.\scripts\start_coppelia_sim.ps1 .\scense\Robobo_Scene.ttt
```

Once all this is started, you can use the run scripts to run the example with the `--simulation` flag:

```sh
bash ./scripts/run.sh --simulation
```

```ps1
.\scripts\run.ps1 --simulation
```

## Project structure

Everything here is structured as follows (`tree -a --dirsfirst`):

```
├── catkin_ws
│   ├── src
│   │   ├── learning_machines
│   │   │   ├── scripts
│   │   │   │   └── learning_robobo_controller.py
│   │   │   ├── src
│   │   │   │   └── learning_machines
│   │   │   │       ├── __init__.py
│   │   │   │       └── test_actions.py
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   └── setup.py
│   │   ├── robobo_interface
│   │   │   ├── src
│   │   │   │   └── robobo_interface
│   │   │   │       ├── utils
│   │   │   │       │   ├── __init__.py
│   │   │   │       │   └── sets.py
│   │   │   │       ├── base.py
│   │   │   │       ├── datatypes.py
│   │   │   │       ├── hardware.py
│   │   │   │       ├── __init__.py
│   │   │   │       └── simulation.py
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   │   └── setup.py
│   │   ├── learning_machines_prey
│   │   │   └── ...
│   │   ├── coppelia_sim
│   │   │   └── ...
│   │   ├── data_files
│   │   │   └── ...
│   │   └── robobo_msgs
│   │       └── ...
│   └── .catkin_workspace
├── models
│   ├── robobo-pusher.ttm
│   └── robobo.ttm
├── scenes
│   ├── arena_approach.ttt
│   ├── arena_obstacles.ttt
│   ├── arena_push_easy.ttt
│   ├── arena_push_hard.ttt
│   └── Robobo_Scene.ttt
├── scripts
│   ├── convert_line_endings.py
│   ├── entrypoint.bash
│   ├── run.ps1
│   ├── run.sh
│   ├── setup.bash
│   ├── start_coppelia_sim.ps1
│   └── start_coppelia_sim.sh
├── results
│   └── ...
├── Dockerfile
└── requirements.txt
```

All the scripts in `./script` should be familiar, with the `convert_line_endings.py` from the [docker tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/docker_tutorial), `start_coppelia_sim.*` from the [CoppeliaSim tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/coppelia_sim_tutorial), the `entrypoint.bash` explained in the [basic_ros_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_basic_setup) tutorial and the `setup.bash` and `run.*` from that same tutorial you used to run the code. `results/` should also be familiar to you, it is the exact same as it was in `basic_ros_setup`.

The scenes and models are kind of new. You'll be familiar with the concept of them from the CoppeliaSim tutorial, but there are more of them now. The new different ones each are for the different assignments of this course. You can look around in them if you want.

The many packages in `catkin_ws` will be unfamiliar, however. In [basic_ros_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_basic_setup) you saw one or two, but, now, there are a ton more. First of all, I should tell you that for most of these, you don't have to open them or understand them at all. They just exist. The two you should pay attention to are `robobo_interface` and `learning_machines`.

`learning_machines` is the package you are expected to work in. You'll end up removing all the code that is there in favor of the stuff you need for the assignments. Currently, it contains the code needed for the little movement you saw when running the code, which you can remove when working on the project. It has the exact same structure as the [basic_ros_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_basic_setup), but this time **the script that is being run is `learning_robobo_controller.py`, and the directory you should write all your actual code in is `src/learning_machines`, with an example file at `test_actions.py`.**

`robobo_interface` contains, well, all the code that is the reason for the fact that we are running in Docker and using CoppeliaSim and all that. You don't need to edit this code, or even understand it, but you have to use it. The code is documented with docstrings inside the files themselves, but I'll still give you a quick overview:

- `hardware.py` contains the final layer of code that talks to the actual robot. It exports one class, `HardwareRobobo`, which, when instantiated, has functions like `move` and `read_orientation`, which you can call to do stuff with the robot.
- `simulation.py` contains the final layer of code that talks to the robot in CoppeliaSim. It exports one class `SimulationRobobo`, which, when instantiated, has most of the same functions as `HardwareRobobo` plus some more. Most importantly, functions like `start_simulation` and `stop_simulation` work with the simulation itself, but also functions like `nr_food_collected` for assignment 3.
- `base.py` contains an interface (or, technically, a template method pattern, but who cares), `IRobobo` to abstract over hardware and software. You'll find abstract definitions of all functions that are on both the hardware and the software and a few template methods like `move_blocking` that work for both with a generic implementation. These are all the functions that work for both hardware and software, and so, these are the functions you should use when training your robot, to make sure that, at least in theory, the behavior of your real Robobo will be similar to the one in the simulation.

If you understood all this, you know everything you should know, and can now get actually finally started with the course itself.

### Troubleshooting

- My smartphone does not connect to the wifi or has a limited connection -> try to update the date/time of the phone.
- My phone’s Bluetooth keeps disconnecting from the Robobo -> this indeed sometimes happens. Maybe you can make a script to deal with it.
- My script keeps trying to connect to the physical Robobo, but nothing happens -> check if the `ROS_MASTER_URI` in `scripts/setup.bash` matches your phone’s current/updated IP address.
- The infrared sensors of my Robobo keep constantly receiving info, even though nothing is touching it -> they may be broken. Get a new Robobo with your supervisor
- My script keeps trying to connect to CoppeliaSim, but nothing happens -> check if your code matches your machine’s current/updated IP address. For this, You want your local IP, usually starting with 192.168, following RFC1918

```ps1
(Get-NetIPAddress | Where-Object { $_.AddressState -eq "Preferred" -and $_.ValidLifetime -lt "24:00:00" }).IPAddress
```

```sh
hostname -I | awk '{print $1}'
```
