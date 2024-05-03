## Instructions for Installing and Running the Simulator

Follow these steps to install and run the robot simulator:

1. **Clone the Repository:**
   ```shell
   git clone https://github.com/ci-group/learning_machines_robobo.git
   cd learning_machines_robobo
   ```

2. **Install Python 3.8:**
   - To ensure compatibility, use Python 3.8. If you don't have Python installed, consider using Anaconda.
    - If using Anaconda, download it [here](https://www.anaconda.com/download).
    
    ```shell
    conda create -n learning_machines python=3.8
    conda activate learning_machines
    conda install -c conda-forge numpy
    conda install -c conda-forge opencv
    ```

    - For general Python users:

    ```shell
    python3.8 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
    ```

3. **Install Docker Desktop:**
   - Refer to [the docs](https://docs.docker.com/desktop/install/mac-install/) for assistance.
   - If you have an apple scillicon mac, make sure to enable expirimental features in the Docker Desktop settings, and, under "Resources", set the amount of CPUs (which is to say, nr of cores) Docker is allowed to use to 1 (Note: This might not be needed on your machine, but 1 is the only value at which it is fully stable on all systems.)

4. **Setup CoppeliaSim:**
   - Download the educational version of CoppeliaSim from their [website](https://www.coppeliarobotics.com/downloads).
    - Make sure to download the version for your hardware (intel mac or M-chip mac).
    - Copy or move the `.app` to `learning_machines_robobo/examples/full_project_setup/coppeliaSim.app`, which you should be able to click-to-run.
    - Your PC might tell you it cannot verify the integrety of the app and refuse to run it. To give it permissions anyway, open finder in the current directory (with `open .`), and control-click the application to show the menu that lets you overwrite these settings.

## Running the Simulator

Once everything is downloaded, you can start the simulator:

### 1. Update `scripts/setup.bash` with Your IP Address

On your terminal, run:

```shell
ipconfig getifaddr en1
```

Update the line in `scripts/setup.bash`:

```bash
export COPPELIA_SIM_IP="your.ip.address"
```

### 2. Start CoppeliaSim

Assuming you are currently in `learning_machines_robobo/examples/full_project_setup/`, run the following command on your terminal (with the venv active):

```shell
zsh ./scripts/start_coppelia_sim.zsh ./scenes/Robobo_Scene.ttt
```

*Note: The scene that CoppeliaSim starts with is specified here.*

### 3. Running the Code

- Launch Docker Engine.
- Depending on if you have an Apple Scillicon mac or an Intel mac:

```shell
# Running on an Intel mac
bash ./scripts/run.sh --simulation

# Running on an Apple Scillicon mac
zsh ./scripts/run_apple_sillicon.zsh --simulation
```

*Note: The executed code is located at `full_project_setup/catkin_ws/src/learning_machines/scripts/learning_robobo_controller.py` and `full_project_setup/catkin_ws/src/learning_machines/src/learning_machines/test_actions.py`. *

*Note: The docker build takes a while. After it successfully runs, you should see the Robobo move the phone, and the following in your terminal:*

<p allign="center">
  <img src="./assets/resulting_print.png" />
</p>

Congratulations! You've successfully completed the setup. 