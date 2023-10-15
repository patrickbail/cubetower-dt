# Cubetower-DT
## Setup
### Prerequisites
* [Omniverse Isaac Sim 2022.2.1](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html)
* [ROS 2](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html) and [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge) (works only on Linux and is only required if extracting and parsing data of ros bags is needed, see instructions bellow)

### Clone repository
* Linux
```
cd ~
git clone https://sci-git.cs.rptu.de/p_noras19/cubetower-dt.git
```
* Windows
```
cd %USERPROFILE%
git clone https://sci-git.cs.rptu.de/p_noras19/cubetower-dt.git
```
## Generating Synthetic Data from Real-world Scenarios with Nvidia's Omniverse Isaac Sim
This project offers means in generating synthetic data from a recreated virtual environment with Isaac Sim. Scene, sensor and trajectory files are already provided to synthetsize camera and LiDAR data from a cubetower setup.\
However, the project was created to generate synthetic data from any given scenario. This means, by changing the USD scene and robot stage, together with the camera or LiDAR specifications any kind of scenario can be used to synthesize data with Isaac Sim. By additionally using own trajectory and sensor timestamp data (no matter if collected from a real scan or built from scratch), even own paths can be taken and interpolated. 
## Further instructions
1. See [Run Standalone](run_standalone.md) for instructions on how to run and work on standalone Isaac Sim python scripts
2. See [Virtual Lab Run](virtual_lab_run.md) for instructions on how to simulate a real-world scenario and how to generate synthetic data
3. See [Extratcing Data](extracting_data.md) for instructions on how to build ROS2 Foxy on your system and how to extract recorded data