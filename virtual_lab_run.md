# Generating Synthetic Data from a Virtual Lab Run
## Setup
### Workstation
Copy over the provided `isaacGenSynData` directory into the Isaac Sim directory
* Linux
```
export ISAAC_SIM="$HOME/.local/share/ov/pkg/isaac_sim-2022.1.1"
cp -RT ~/Isaac-Sim-Playground/isaacGenSynData $ISAAC_SIM
```
* Windows
```
set ISAAC_SIM="%LOCALAPPDATA%\ov\pkg\isaac_sim-2022.1.1"
copy /Y /I %USERPROFILE%\Isaac-Sim-Playground\isaacGenSynData %ISAAC_SIM%\isaacGenSynData
```
### Docker
Run the following docker command to start the Isaac Sim container with the codebase:
```
docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \ 
-v ~/Desktop/Isaac-Sim-Playground:/isaac-sim/Isaac-Sim-Playground \ 
-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache/Kit:rw \
-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \ 
-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
-v ~/docker/isaac-sim/documents:/root/Documents:rw \
nvcr.io/nvidia/isaac-sim:2022.2.1
```
Inside the docker container execute the following script to setup the environmnet:
```
./Isaac-Sim-Playground/install/debug_install.sh
```

The `isaacGenSynData` directory contains `parse_bag.py`, `virtual_lab_run.py`, `lab_utility.py` and two JSON files `path_test.json` and `lerp_test.json`. The `path_test.json` is a demonstration of a trajectory file that `virtual_lab_run.py` needs as an input with the -p flag. The simulator will follow the path inside the provided JSON file. In addition, a file `lerp_test.json` is available as a template for testing linear interpolation between two points, if a path is not needed for the simulation.

## Start a Virtual Lab Run and generate synthetic data
Simulating the real-world scenario can be done by running the python script like every other standalone Isaac Sim python script, which was demonstraed in the [Run Standalone](run_standalone.md) section. \
Run a simulation via:
* Linux
```
./python.sh ./Isaac-Sim-Playground/isaacGenSynData/virtual_lab_run.py -p <path_to_json> [-i] [-l] [-s]
```
* Windows
```
./python.bat .\Isaac-Sim-Playground\isaacGenSynData\virtual_lab_run.py -p <path_to_json> [-i] [-l] [-s]
```

Arguments:
* -p | --path: A .json file that specifies the path the robot takes. Extracted path or pose data generated from `parse_bag.py` belongs here
* -i | --interpolate: If provided a .json file of recorded image or pointcloud data with timestamps then interpolate missing poses. Extracted data generated from `parse_bag.py` belongs here
* -l | --lidar: If specified, a lidar scan will be done and a registered pointcloud will be saved in `_out_pcd`
* -s | --stereo: If specified, a scan with stereo vision will be done and images at every step will be saved in `_out_image` 
* -L | --lerp: If specified, a linear interpolation between two poses will be done, instead of using the path provided via the -p argument
* -E | --early_stopping: If specified a value, simulation will be stopped early according to the provided value
* --livestream: Starts a livestream service, such that a Streaming Client can connect to a remote Isaac Sim instance
