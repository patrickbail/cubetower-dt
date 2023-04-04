# Generating Synthetic Data from a Virtual Lab Run
## Setup
Copy over the provided `lab_run` directory into the Isaac Sim directory
* Linux
```
export ISAAC_SIM="$HOME/.local/share/ov/pkg/isaac_sim-2022.1.1"
cp -RT ~/Isaac-Sim-Playground/lab_run $ISAAC_SIM
```
* Windows
```
set ISAAC_SIM="%LOCALAPPDATA%\ov\pkg\isaac_sim-2022.1.1"
copy /Y /I %USERPROFILE%\Isaac-Sim-Playground\lab_run %ISAAC_SIM%\lab_run
```
The `lab_run` directory contains `parse_bag.py`, `virtual_lab_run.py` and two USD files for the virtual enviroment `acoposobj_scaled.usd` and `ZED2_scaled.usd` which need to be in the same directory as `virtual_lab_run.py`.
## Parse and Extract important Data
To generate synthetic data in the Virtual Lab with Isaac Sim, we first need to extract and parse recorded ROS2 bags.
This can be done with `parse_bag.py` by specifying which bag and what topic should be parsed.

Parse a bag file via:
`./parse_bag.py -b [PATH_TO_BAG] -t [TOPIC_NAME]`

Currently only topics of these types are supported:
* nav_msgs/msg/Path
* geometry_msgs/msg/PoseStamped
* sensor_msgs/msg/Image
* sensor_msgs/msg/PointCloud2

ROS2 has to be installed on the system, follow these installation steps if this is not the case: https://docs.ros.org/en/foxy/Installation.html \
Setup the ROS2 enviroment first before running the script via: \
`. ~/ros2_foxy/install/local_setup.bash`

## Start a Virtual Lab Run
Simulating the real-world scenario can be done by running the python script like every other standalone Isaac Sim python script. See [Run Standalone](run_standalone.md) \
Run a simulation via:
* Linux
`./python.sh ./lab_run/virtual_lab_run.py -p [PATH] -s [PATH] (-l) (-d)`
* Windows
`./python.bat .\lab_run\virtual_lab_run.py -p [PATH] -s [PATH] (-l) (-d)`

Arguments:
* -p / --path: A .json file that specifys the pathmap. Extracted data generated from `parse_bag.py` belongs here
* -s / --sim_data: A .json file of recorded image or pointcloud data with timestamps. Extracted data generated from `parse_bag.py` belongs here
* -l / --lidar: If specified, a lidar scan will be done
* -d / --depth_map: If specified, a depth map will be generated
