# Full project example

_Note: to quickly install everything for this project (For example, to run on another machine), go to `quick_setup_[your_os].md` for the checklist._

This is the example you are expected to use as a project template. The structure is the same as [basic_ros_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_basic_setup), it contains the same dockerfile and some of the same scripts and packages, but it contains a lot more stuff on top of that. First of all, it contains the same `start_coppelia_sim` script as [coppelia_sim_tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/coppelia_sim_tutorial), meaning you should also copy-past CoppeliaSim to here, such that `./CoppeliaSim` (or `coppeliaSim.app`) exists. It also contains the `setup.bash` from [hardware_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/hardware_setup) that you should configure to have the `ROS_MASTER_URI`.

## Running the code that is there

As you find it, this example is a fully working system. Currently, it contains a script that simply connects to the robot, be it hardware or software, and then runs through the motions it supports: moving itself and the phone around, getting input from the camera and the IR sensors, and a whole bunch more.

If you're on Apple Silicon you need to do one more thing, as that is an ARM CPU architecture instead of x86. You can read about this in the [docker_tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/docker_tutorial#running-with-apple-sillicon). After you've done that, you should use the `_apple_sillicon` commands. Intel macs are unaffected.

### Running with simulation

#### Starting CoppeliaSim

To run it with the simulation, you have to, first of all, make sure that CoppeliaSim is installed under `./CoppeliaSim` or `./coppeliaSim.app` like in the [coppelia_sim_tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/coppelia_sim_tutorial). After that, you have to update `setup.bash`, updating `COPPELIA_SIM_IP` with the IP of the computer you're running CoppeliaSim on, which is to say your own. We have to do this because the container (again, think of it as a small separate computer), has to connect to your computer to find it. Technically, though, this also means you can run the simulation on another system as this package.

To get your own IP address, you have to run `Get-NetIPAddress` on Windows PowerShell, but this gives you a bunch of extra stuff. To get _only_ your IP address, you can run:

```ps1
(Get-NetIPAddress | Where-Object { $_.AddressState -eq "Preferred" -and $_.ValidLifetime -lt "24:00:00" }).IPAddress
```

similarly, on Linux, you can run `hostname -I` to get all the information, but the following to get only the IP you are looking for:

```sh
hostname -I | awk '{print $1}'
```

Finally, on macOS, you can run `ifconfig` for all information, and (presuming you only have one network card on your laptop), this to get only what you are looking for:

```zsh
ifconfig en0 | awk '/inet / {print $2}'
```

If this command does not work on Mac, you can go to wifi settings -> network settings (under the "more" tab of the network), which will show you your IP address.

After you have updated `./scripts/setup.bash` you can start CoppeliaSim with the `start_coppelia_sim.sh` script like in that example:

```sh
# Linux
bash ./scripts/start_coppelia_sim.sh ./scenes/Robobo_Scene.ttt
```

```zsh
# MacOS
zsh ./scripts/start_coppelia_sim.zsh ./scenes/Robobo_Scene.ttt
```

```ps1
# Windows
.\scripts\start_coppelia_sim.ps1 .\scenes\Robobo_Scene.ttt
```

#### Running the code itself inside docker

Once all this is started, you can use the run scripts to run the example with the `--simulation` flag:

```sh
# Linux, or mac with Intel processor
bash ./scripts/run.sh --simulation
```

```ps1
# Windows
.\scripts\run.ps1 --simulation
```

Or, if you're on a mac _with Apple Sillicon_ (other mac users can just use `run.sh`)

```zsh
# Mac with Apple Sillicon
zsh ./scripts/run_apple_sillicon.zsh --simulation
```

### Running with hardware

To run it with hardware, you have to set everything up in the same way as the [hardware_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/hardware_setup), which is to say you have to confirm you're on the same (non-public) network with the phone and your laptop/desktop, and have to put the `ROS_MASTER_URI` (`http://[Adress shown on top left]:11311`) in `setup.bash`.

After this, you can run with the flag `--hardware`. If you did everything correctly, you'll then see the robot move around and do a bunch of stuff.

```sh
# Linux, or mac with Intel processor
bash ./scripts/run.sh --hardware
```

```ps1
# Windows
.\scripts\run.ps1 --hardware
```

Or, if you're on a mac _with Apple Sillicon_ (other mac users can just use `run.sh`)

```zsh
# Mac with Apple Sillicon
zsh ./scripts/run_apple_sillicon.zsh --hardware
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
│   ├── entrypoint.bash
│   ├── run_apple_sillicon.zsh
│   ├── run.ps1
│   ├── run.sh
│   ├── setup.bash
│   ├── start_coppelia_sim.ps1
│   ├── start_coppelia_sim.sh
│   └── start_coppelia_sim.zsh
├── results
│   └── ...
├── Dockerfile
└── requirements.txt
```

### Scripts

All the scripts in `./script` should be familiar, with the, `start_coppelia_sim.*` from the [CoppeliaSim tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/coppelia_sim_tutorial), the `entrypoint.bash` explained in the [basic_ros_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_basic_setup) tutorial and the `setup.bash` and `run.*` from that same tutorial you used to run the code. `results/` should also be familiar to you, it is the exact same as it was in `basic_ros_setup`.

### Models

The scenes and models are kind of new. You'll be familiar with the concept of them from the CoppeliaSim tutorial, but there are more of them now. The new different ones each are for the different assignments of this course. You can look around in them if you want.

### Catkin Workspace

The many packages in `catkin_ws` will be unfamiliar, however. In [basic_ros_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_basic_setup) you saw one or two, but, now, there are a ton more. First of all, I should tell you that for most of these, you don't have to open them or understand them at all. They just exist. The two you should pay attention to are `robobo_interface` and `learning_machines`.

#### Where your code will live

`learning_machines` is the package you are expected to work in. You'll end up removing all the code that is there in favor of the stuff you need for the assignments. Currently, it contains the code needed for the little movement you saw when running the code, which you can remove when working on the project. It has the exact same structure as the [basic_ros_setup](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/ros_basic_setup), but this time **the script that is being run is `learning_robobo_controller.py`, and the directory you should write all your actual code in is `src/learning_machines`, with an example file at `test_actions.py`.**

#### The API for working with the robot

`robobo_interface` contains, well, all the code that is the reason for the fact that we are running in Docker and using CoppeliaSim and all that. You don't need to edit this code, or even understand it, but you have to use it. The code is documented with docstrings inside the files themselves, but I'll still give you a quick overview:

- `hardware.py` contains the final layer of code that talks to the actual robot. It exports one class, `HardwareRobobo`, which, when instantiated, has functions like `move` and `read_orientation`, which you can call to do stuff with the robot.
- `simulation.py` contains the final layer of code that talks to the robot in CoppeliaSim. It exports one class `SimulationRobobo`, which, when instantiated, has most of the same functions as `HardwareRobobo` plus some more. Most importantly, functions like `start_simulation` and `stop_simulation` work with the simulation itself, but also functions like `nr_food_collected` for assignment 3.
- `base.py` contains an interface (or, technically, a template method pattern, but who cares), `IRobobo` to abstract over hardware and software. You'll find abstract definitions of all functions that are on both the hardware and the software and a few template methods like `move_blocking` that work for both with a generic implementation. These are all the functions that work for both hardware and software, and so, these are the functions you should use when training your robot, to make sure that, at least in theory, the behavior of your real Robobo will be similar to the one in the simulation.

### Dockerfile / requirements.txt

Lastly, there is the `Dockerfile` and the `requirements.txt`. In the dockerfile, you are going to see quite a lot of things that you haven't seen before. However, you have to edit these dockerfiles very little (in fact, you can completely avoid editing them without TA instruction if you want), and every line has a comment above it, so it should all be clear. Secondly, the `requrements.txt` file contains the Python requirements for your Python code (e.g. pandas, keras, tensorflow, whatever). The requirements specified here will be installed in the docker container. If you have never used a file like this before, you can install all things specified in it yourself with `pip install -r requirements.txt`, and specify dependency versions using [python version specifiers](https://packaging.python.org/en/latest/specifications/version-specifiers/), just make sure you're specifying python 3.8 compatible versions of packages.

If you understood all this, you know everything you should know, and can now get actually finally started with the course itself.

## Looking ahead

A lot more things are possible with this code that is not specified in this readme. Here, we provide some code snippets that you might like to look at to see what is possible.

### Running multiple instances of the Simulation.

When training your models, you might like to run multiple instances of CoppeliaSim at a time, each of which to be connected to by a different instance of `SimulationRobobo`, which gives a performance increase when you have a lot (>8) of CPU cores available.

For this, you first want to start the different instances of CoppeliaSim. For this, you can run the `start_copplia_sim` script multiple times:

for example, on Linux:

```sh
# One terminal
bash ./scripts/run_coppelia_sim.sh ./scenes/Robobo_Scene.ttt 20000 -h

# In another terminal
bash ./scripts/run_coppelia_sim.sh ./scenes/Robobo_Scene.ttt 20001 -h

# ... etc
```

On Windows:

```ps1
# One terminal
.\scripts\run_coppelia_sim.ps1 .\scenes\Robobo_Scene.ttt 20000 -headless

# In another terminal
.\scripts\run_coppelia_sim.ps1 .\scenes\Robobo_Scene.ttt 20001 -headless

# ... etc
```

On macOS:

```sh
# One terminal
zsh ./scripts/run_coppelia_sim.zsh ./scenes/Robobo_Scene.ttt 20000 -h

# In another terminal
zsh ./scripts/run_coppelia_sim.zsh ./scenes/Robobo_Scene.ttt 20001 -h

# ... etc
```

Let's say you started three instances of CoppeliaSim, at ports 20000, 20001, and 20002.

After this, you can make your main Python script (`learning_robobo_controller.py`) into something like this:

```python
#!/usr/bin/env python3
import multiprocessing

from robobo_interface import SimulationRobobo

# The same default function that was there when you first pulled
from learning_machines import run_all_actions


def run(i: int) -> None:
    rob = SimulationRobobo(api_port=(20000 + i))
    run_all_actions(rob)


if __name__ == "__main__":
    with multiprocessing.Pool(3) as p:
        p.map(run, range(3))
```

This will use three separate Python threads / processes, to connect to the three CoppeliaSim API instances, and run the `run_all_actions` function in those threads.

You can run this with your OS's equivalent of `./scripts/run.sh` without any further arguments (as we don't parse any in Python's `__main__`)

### Speeding up build times by caching the C++ stages

Using [Docker's multi-stage builds feature](https://docs.docker.com/build/building/multi-stage/), it is possible to take advantage of Docker's caching functionality to get incremental compiles to work, meaning you won't have to re-compile the (presumably untouched) C++ code every time you run `docker build`. Tough this dockerfile is not the default for the full project setup (as it's likely to be somewhat overwhelming,) you can make use of it.

To get access to this, you have to re-build the examples (as is the case for all "advanced usage" sections,) This deletes any changed files inside the `examples/` directory, so make sure you don't have any files saved here before running this. To re-build cd into `maintained/` of this project, and run `python3 build.py --cached_cpp_builds` from the `maintained/` directory.

If you cd back into the full project setup after this, you will see the Dockerfile is replaced with a new, longer one that does roughly the same things, but in a multi-stage process.

## Troubleshooting

- My smartphone does not connect to the wifi or has a limited connection -> try to update the date/time of the phone. Also, make sure you're on a somewhat private network (eduroam does not work.)
- My phone’s Bluetooth keeps disconnecting from the Robobo (to re-connect after a while) -> this indeed sometimes happens. Maybe you can make a script to deal with it.
- My script keeps trying to connect to the physical Robobo, but nothing happens -> check if the `ROS_MASTER_URI` in `scripts/setup.bash` matches your phone’s current/updated IP address.
- The infrared sensors of my Robobo keep constantly receiving info, even though nothing is touching it -> they may be broken. Get a new Robobo with your supervisor
- My script keeps trying to connect to CoppeliaSim, but nothing happens -> check if your code matches your machine’s current/updated IP address. For this, You want your local IP, how to find your IP is listed [here](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/full_project_setup#running-with-simulation). It might also be an issue with unstable IP addresses (which is to say eduroam), so connecting to a private network might also help.
- Stuff throws an error, but I cannot find why because it's all in the black box that is Docker. -> In the [docker_tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/docker_tutorial), it is shown how to run docker in "interactive" mode. To do that for this example, you should change the `run` script (and the Dockerfile) to start the container like that.
- I want to start a new docker container, but another one is still running because it crashed. How do I close it? -> You need to run `docker container stop [id]`, you can read up on this in the [docker tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/docker_tutorial#managing-running-containers)
- Docker is taking up way too much space / RAM -> On Windows and MacOS, it is expected that Docker uses a few dozen gigs of storage and about 10G ram when available. That's just because it's a separate computer, with its own operating system and everything. However, it can sometimes be good to run `docker container prune`, as described in the [docker tutorial](https://github.com/ci-group/learning_machines_robobo/tree/master/examples/docker_tutorial#deleting-built-images)
