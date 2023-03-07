# Running standalone Isaac Sim Python scripts in Visual Studio Code
## Setup
### Prerequisites
* [Omniverse Isaac Sim 2022.1.1](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html)
* [Visual Studio Code](https://code.visualstudio.com/)
### 1. Clone repository
* Linux
```
cd ~
git clone https://ifs.dfki.de/umgebungsmodellierung/scene-reconstruction/Isaac-Sim-Playground.git
```
* Windows
```
cd %USERPROFILE%
git clone https://ifs.dfki.de/umgebungsmodellierung/scene-reconstruction/Isaac-Sim-Playground.git
```
### 2. Patch Isaac Sim 2022.1.1 .vscode directory
Copy over the provided `.vscode` directory into the Isaac Sim `.vscode` directory
* Linux
```
export ISAAC_SIM="$HOME/.local/share/ov/pkg/isaac_sim-2022.1.1"
cp -RT ~/Isaac-Sim-Playground/.vscode $ISAAC_SIM
```
* Windows
```
set ISAAC_SIM="%LOCALAPPDATA%\ov\pkg\isaac_sim-2022.1.1"
copy /Y /I %USERPROFILE%\Isaac-Sim-Playground\.vscode %ISAAC_SIM%\.vscode
```
## Running and working with standalone Isaac Sim Python scripts
### Working with the Core API in Visual Studio Code
After patching the VSCode workspace of the Isaac Sim package, to get Autocomplete and IntelliSense while working with the Isaac Sim Core API inside VSCode, open the folder inside VSCode where the `.vscode` directory is located.

Every standalone Python script that makes use of the Omniverse API, needs to be inside the Isaac Sim directory to get IntelliSense and Autocomplete while working on it.

More details about the pre-configured enviroment can be looked up at the [Omniverse Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/manual_standalone_python.html)
### Launching a standalone Isaac Sim Python script
If Assets need to be loaded fron the Omniverse API, then an access to a Nucleus database, either a connection to the local Nucleus service or a remote one, has to be established (Can be done by starting the Omniverse Launcher). Only then Assets, like a ground plane and more, can be found and loaded into the simulation.

Run a standalone Isaac Sim Python script via:
* Linux `./python.sh [PATH]/[PYTHON_SCRIPT].py`
* Windows `./python.bat [PATH]\[PYTHON_SCRIPT].py`

To test if it works, try running: 
* Linux
`./python.sh ./standalone_examples/api/omni.isaac.core/simulate_robot.py`
* Windows
`./python.bat .\standalone_examples\api\omni.isaac.core\simulate_robot.py`