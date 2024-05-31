## Instructions for Installing and Running the Simulator

Follow these steps to install and run the robot simulator:

1. **Clone the Repository:**

   ```shell
   git clone https://github.com/ci-group/learning_machines_robobo.git
   cd learning_machines_robobo
   ```

2. **Install Python 3.8:**

   - You can use specific Python versions with `pyenv` or `brew`, depending on how you set up Python.

     - Note that annaconda (and other distributions which do more than just shill the executable,) may not work.

   - Create a venv, and install dependencies:

   ```shell
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```

3. **Install Docker Desktop:**

   - Refer to [the docs](https://docs.docker.com/desktop/install/mac-install/) for assistance.
   - If you have an Apple silicon mac, make sure to enable experimental features in the Docker Desktop settings, and, under "Resources", set the amount of CPUs (which is to say, nr of cores) Docker is allowed to use to 1 (Note: This might not be needed on your machine, but 1 is the only value at which it is fully stable on all systems.)

4. **Setup CoppeliaSim:**
   - Download the educational version of CoppeliaSim from their [website](https://www.coppeliarobotics.com/downloads).
   - Make sure to download the version for your hardware (Intel mac or M-chip mac).
   - Copy or move the `.app` to `learning_machines_robobo/examples/full_project_setup/coppeliaSim.app`, which you should be able to click-to-run.
   - Your PC might tell you it cannot verify the integrity of the app and refuse to run it. To give it permissions anyway, open finder in the current directory (with `open .`), and control-click the application to show the menu that lets you overwrite these settings.

## Running the Simulator

Once everything is downloaded, you can start the simulator:

### 1. Update `scripts/setup.bash` with Your IP Address

On your terminal, run:

```shell
ipconfig getifaddr en0
```

Update the line in `scripts/setup.bash`:

```bash
export COPPELIA_SIM_IP="your.ip.address"
```

This command might not work, if it doesn't, find your IP another way. Google will come up with several options, at least one of which should work.

### 2. Start CoppeliaSim

Assuming you are currently in `learning_machines_robobo/examples/full_project_setup/`, run the following command on your terminal (with the venv active):

```shell
zsh ./scripts/start_coppelia_sim.zsh ./scenes/Robobo_Scene.ttt
```

_Note: The scene that CoppeliaSim starts with is specified here._

### 3. Running the Code

- Launch Docker Engine.
- Depending on if you have an Apple Scillicon mac or an Intel mac:

```shell
# Running on an Intel mac
bash ./scripts/run.sh --simulation

# Running on an Apple Scillicon mac
zsh ./scripts/run_apple_sillicon.zsh --simulation
```

_Note: The executed code is located at_ `full_project_setup/catkin_ws/src/learning_machines/scripts/learning_robobo_controller.py` _and_ `full_project_setup/catkin_ws/src/learning_machines/src/learning_machines/test_actions.py`.

_Note: The docker build takes a while. After it successfully runs, you should see the Robobo move the phone, and the following in your terminal:_

<p allign="center">
  <img src="./assets/resulting_print.png" />
</p>

Congratulations! You've successfully completed the setup.
