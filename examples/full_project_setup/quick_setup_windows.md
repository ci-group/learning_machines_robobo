## Instructions for Installing and Running the Simulator

Follow these steps to install and run the robot simulator:

1. **Clone the Repository:**

   ```pwsh
   git clone https://github.com/ci-group/learning_machines_robobo.git
   cd learning_machines_robobo
   ```

2. **Install Python 3.8:**

   - To ensure compatibility, use Python 3.8. You can install this separately, and then run with the `py` launcher.

   - Create a venv, and install the dependencies:

   ```powershell
   py -3.8 -m venv .venv
   .venv\Scripts\Activate.ps1
   py -3.8 -m pip install -r requirements.txt
   ```

3. **Install Docker (for Windows):**

   - Install WSL2.
   - Enable hardware virtualization.
   - Refer to [the docs](https://docs.docker.com/desktop/install/windows-install/) for assistance.

4. **Setup CoppeliaSim:**
   - Download the educational version of CoppeliaSim from their [website](https://www.coppeliarobotics.com/downloads).
   - Make sure to download the zip version for Windows.
   - Extract the contents to `learning_machines_robobo\examples\full_project_setup\CoppeliaSim`.

## Running the Simulator

Once everything is downloaded, you can start the simulator:

### 1. Update `scripts\setup.bash` with Your IP Address

On Windows PowerShell, run:

```powershell
(Get-NetIPAddress | Where-Object { $_.AddressState -eq "Preferred" -and $_.ValidLifetime -lt "24:00:00" }).IPAddress
```

Update the line in `scripts\setup.bash`:

```bash
export COPPELIA_SIM_IP="your.ip.address"
```

### 2. Start CoppeliaSim

Assuming you are currently in `learning_machines_robobo\examples\full_project_setup\`, run the following command in PowerShell (with the venv active):

```powershell
.\scripts\start_coppelia_sim.ps1 .\scenes\Robobo_Scene.ttt
```

_Note: The scene that CoppeliaSim starts with is specified here._

### 3. Running the Code

- Launch Docker Engine.
- On PowerShell, run:

```powershell
.\scripts\run.ps1 --simulation
```

_Note: The executed code is located at_ `full_project_setup\catkin_ws\src\learning_machines\scripts\learning_robobo_controller.py` _and_ `full_project_setup\catkin_ws\src\learning_machines\src\learning_machines\test_actions.py`.

_Note: The docker build takes a while. After it successfully runs, you should see the Robobo move the phone, and the following in your terminal:_

<p allign="center">
  <img src="./assets/resulting_print.png" />
</p>

Congratulations! You've successfully completed the setup.
