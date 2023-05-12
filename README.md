# Generating Synthetic Data from Real-world Scenarios with Nvidia's Omniverse Isaac Sim
## Setup
### Prerequisites
* [Omniverse Isaac Sim 2022.1.1](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html)
* [Visual Studio Code](https://code.visualstudio.com/) (only needed if working on Isaac sim standalone scritps is required, see instructions below)
* [ROS2](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html) and [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge) (works only on Linux and is only required if extracting and parsing data of ros bags is needed, see instructions bellow)

### Clone repository
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

## Further instructions
1. See [Run Standalone](run_standalone.md) for running and working on standalone Isaac Sim python scripts with Visual Studio Code
2. See [Virtual Lab Run](virtual_lab_run.md) to simulate a real-world scenario and to generate synthetic data
3. See [Extratcing Data](extracting_data.md) for instructions on how to build ROS2 foxy on your system and how to extract recorded data