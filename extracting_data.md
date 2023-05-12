# Building ROS2 foxy and extracting data from ROS/ROS2 bags
## Setup
### Build ROS2 foxy on Linux
ROS2 foxy and cv_bridge have to be build/installed on the system. \
Execute the ```build_ros2.sh``` script to build ROS2 foxy and cv_bridge on your Linux system.
You can also build or install ROS2 on your own, although this is not recommended. \
Before running anything that uses ROS2, setup the ROS2 enviroment first via:
```
. ~/ros2_foxy/install/local_setup.bash
```

## Parse and Extract  Data
To generate synthetic data in the Virtual Lab with Isaac Sim and later analyse the accuracy of the generated synthetic data vs. real recorded data, we first need to extract and parse recorded ROS2 bags.
This can be done with `parse_bag.py` by specifying which bag and what data from which topic should be extracted.

Parse a bag file with a specific topic via:
```
./parse_bag.py -b [PATH_TO_BAG] -t [TOPIC_NAME]
```
This will create a .json file and in case of image or pointcloud data, .img/.ply files will be saved in their respective output directories `raw_img`/`raw_pcd`

Currently only topics of these types are supported:
* nav_msgs/msg/Path
* geometry_msgs/msg/PoseStamped
* sensor_msgs/msg/Image
* sensor_msgs/msg/PointCloud2

The following arguments can be used:
* -b | --bag: Path to ROS/ROS2 bag file that will be parsed
* -t | --topic: Name of topic from which data should be extracted
* -n | --name: If specified, name of the outgoing saved .png or .ply files will be changed
* -v | --visualize_path: If specified, pathmap will be visualized
* -e | --export_to_json: If specified, file will not be exported to json