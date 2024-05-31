## Instructions for Installing and Running the Simulator

Follow these steps to install and run the robot simulator:

1. **Clone the Repository:**

   ```shell
   git clone https://github.com/ci-group/learning_machines_robobo.git
   cd learning_machines_robobo
   ```

2. **Install Python 3.8:**

   - To ensure compatibility, use Python 3.8. If you don't have this version installed, do so using your distro's package manager.
   - Create a venv, and install the dependencies.

   ```shell
   python3.8 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```

3. **Install Docker:**

   - You can install Docker Desktop if you really want to, but you only really need to install Docker engine. Find out how to do this for your os [at this doc page](https://docs.docker.com/engine/install/)
   - After this, you can `systemctl enable docker` to start up the docker engine on system startup (Which has minimal overhead, unlike Docker Desktop,) or `systemctl start docker` to start it once.

4. **Setup CoppeliaSim:**
   - Download the educational version of CoppeliaSim from their [website](https://www.coppeliarobotics.com/downloads).
   - Download the version for ubuntu 22.04. You can get it working for most distro's regardless of it being compiled for Ubuntu.
   - Extract the tarball to `learning_machines_robobo/examples/full_project_setup/CoppeliaSim`
   - Troubleshoot your install, especially when not on Ubuntu 22.04. You might either be missing some C libraries, or have the wrong versions. The first stop is `./CoppeliaSim/libLoadErrorCheck.sh`, and your distro's package manager, but, after that, setting `$LD_LIBRARY_PATH` and doing a little `ldd` might also be required.

## Running the Simulator

Once everything is downloaded, you can start the simulator:

### 1. Update `scripts/setup.bash` with Your IP Address

On your terminal, run:

```shell
hostname -I | awk '{print $1}'
```

Update the line in `scripts/setup.bash`:

```bash
export COPPELIA_SIM_IP="your.ip.address"
```

### 2. Start CoppeliaSim

Assuming you are currently in `learning_machines_robobo/examples/full_project_setup/`, run the following command on your terminal:

```shell
bash ./scripts/start_coppelia_sim.sh ./scenes/Robobo_Scene.ttt
```

_Note: The scene that CoppeliaSim starts with is specified here._

### 3. Running the Code

- Start Docker
- Run in your shell:

```shell
# NOTE: `chmod u+x ./scripts/run.sh` to run it without the "bash"
bash ./scripts/run.sh --simulation
```

_Note: The executed code is located at_ `full_project_setup/catkin_ws/src/learning_machines/scripts/learning_robobo_controller.py` _and_ `full_project_setup/catkin_ws/src/learning_machines/src/learning_machines/test_actions.py`.

_Note: The docker build takes a while. After it successfully runs, you should see the Robobo move the phone, and the following in your terminal:_

<p allign="center">
  <img src="./assets/resulting_print.png" />
</p>

Congratulations! You've successfully completed the setup.
