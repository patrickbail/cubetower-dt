# Generating Synthetic Data from a Virtual Lab Run
## Setup
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
The `isaacGenSynData` directory contains `parse_bag.py`, `virtual_lab_run.py` and two USD files `acoposobj_scaled.usd` and `ZED2_scaled.usd` for the virtual enviroment, which need to be in the same directory as `virtual_lab_run.py`.

## Start a Virtual Lab Run and generate synthetic data
Simulating the real-world scenario can be done by running the python script like every other standalone Isaac Sim python script, which was demonstraed in the [Run Standalone](run_standalone.md) section. \
Run a simulation via:
* Linux
```
./python.sh ./isaacGenSynData/virtual_lab_run.py -p <path_to_json> [-i] [-l] [-s]
```
* Windows
```
./python.bat .\isaacGenSynData\virtual_lab_run.py -p <path_to_json> [-i] [-l] [-s]
```

Arguments:
* -p | --path: A .json file that specifies the path the robot takes. Extracted path or pose data generated from `parse_bag.py` belongs here
* -i | --interpolate: If provided a .json file of recorded image or pointcloud data with timestamps then interpolate missing poses. Extracted data generated from `parse_bag.py` belongs here
* -l | --lidar: If specified, a lidar scan will be done and a registered pointcloud will be saved in `_out_pcd`
* -s | --stereo: If specified, a scan with stereo vision will be done and images at every step will be saved in `_out_annot` 
