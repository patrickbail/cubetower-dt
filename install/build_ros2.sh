#!/bin/bash

# Locate and export the repository location
REPO_DIR=$(git rev-parse --show-toplevel | grep "cubetower-dt")
export REPO_DIR

# Check if the repository directory is found
if [ -z "$REPO_DIR" ]; then
  echo "Error: Repository directory not found. Make sure the repository is cloned in the specified folder."
  exit 1
fi

# Check for UTF-8 locale
cd ~
locale

# Install locales package
sudo apt update && sudo apt install locales

# Generate and set UTF-8 locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify locale settings
locale

# Install required packages
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Update package information and install curl
sudo apt update && sudo apt install -y curl

# Download ROS keyring
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 Foxy repository to sources.list.d
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package information and install ROS 2 dependencies
sudo apt update && sudo apt install -y \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

# Install pip packages for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

# Install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

# Install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev

# Create ROS 2 Foxy workspace
mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy

# Fetch ROS 2 Foxy packages
vcs import --input https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos src

# Upgrade existing packages
sudo apt upgrade

# Initialize rosdep
sudo rosdep init
rosdep update

# Install ROS 2 Foxy dependencies
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

# Install additional dependencies for OpenCV
sudo apt install -y python3-numpy libboost-python-dev

# Clone vision_opencv repository
cd ~/ros2_foxy/src
git clone https://github.com/ros-perception/vision_opencv.git -b foxy

# Build ROS 2 Foxy and OpenCV
cd ~/ros2_foxy
colcon build --symlink-install

# Move point_cloud2.py to sensor_msgs package
cd "$REPO_DIR/install"
cp ./point_cloud2.py ~/ros2_foxy/install/sensor_msgs/lib/python3.8/site-packages/sensor_msgs

# Install dependencies
pip install -r requirements.txt

# Source the environment
. ~/ros2_foxy/install/local_setup.bash