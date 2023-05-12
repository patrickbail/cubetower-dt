import os
import cv2
import json
import sqlite3
import argparse
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2

#ROS2 has to be build on the systen, follow installation steps: https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html
#Follow the steps to install cv_bridge https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge
#Setup the ROS2 enviroment first before running the script via: . ~/ros2_foxy/install/local_setup.bash
#~/ros2_foxy/install/sensor_msgs/lib/python3.8/site-packages/sensor_msgs point_cloud2.py

#bag_file = 'zed_kuba_1_0.db3'
#bag_file = '../lidar_kuba_1/lidar_kuba_1_0.db3'
#bag_file = '../lidar_kuba_2/lidar_kuba_2_0.db3'
#bag_file = '../lidar_kuba_3/lidar_kuba_3_0.db3'
#topic_name = '/zed2/zed_node/path_map'
#topic_name = '/zed2/zed_node/pose'
#topic_name = '/zed2/zed_node/left_raw/image_raw_color'
#topic_name = '/zed2/zed_node/right_raw/image_raw_color'
#topic_name = '/zed2/zed_node/depth/depth_registered'
#topic_name = '/rslidar_points'

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    def get_topic_type(self, topic_name):
        return self.topic_type[topic_name]
    
    #[message0, message1, ...]
    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]

        #Get rows from the db
        #rows = self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id} LIMIT 1").fetchall()
        rows = self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id}").fetchall()
        #Deserialise all messages
        #return return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]
        return [deserialize_message(data, self.topic_msg_message[topic_name]) for timestamp, data in rows]

    #Get messages in batches
    #[message0, message1, ...]
    def get_messages_in_batches(self, topic_name):
        offset = 0
        batch_size = 100
        topic_id = self.topic_id[topic_name]

        query = f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id} LIMIT {batch_size} OFFSET {offset}"
        rows = self.cursor.execute(query).fetchall()
        while rows:
            result = [deserialize_message(data, self.topic_msg_message[topic_name]) for timestamp, data in rows]
            yield result

            offset += batch_size
            query = f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id} LIMIT {batch_size} OFFSET {offset}"
            rows = self.cursor.execute(query).fetchall()
        print("Exiting while loop")
        raise Exception
    
    def parse_batches(self, topic_name, file_name):
        topic_type = self.get_topic_type(topic_name)
        total_list = []
        offset = 0
        if (topic_type == "sensor_msgs/msg/Image"):
            # Create directory to save images
            file_path = os.path.join(os.getcwd(), "raw_img", "")
            dir_name = os.path.dirname(file_path)
            os.makedirs(dir_name, exist_ok=True)
            if not file_name:
                file_name = "img"
            try:
                for deserialized_data in self.get_messages_in_batches(topic_name):
                    print(f"Getting Batch {offset}...")
                    parsed_data = self.pares_image(deserialized_data, offset, dir_name, file_name)
                    total_list += parsed_data
                    offset += 100
            except Exception:
                print("Done parsing all batches")
        elif (topic_type == 'sensor_msgs/msg/PointCloud2'):
            # Create directory to save pointcloud data
            file_path = os.path.join(os.getcwd(), "raw_pcd", "")
            dir_name = os.path.dirname(file_path)
            os.makedirs(dir_name, exist_ok=True)
            if not file_name:
                file_name = "pcd"
            try:
                for deserialized_data in self.get_messages_in_batches(topic_name):
                    print(f"Getting Batch {offset}...")
                    parsed_data = self.parse_pointcloud(deserialized_data, offset, dir_name, file_name)
                    total_list += parsed_data
                    offset += 100
            except Exception:
                print("Done parsing all batches")
        return total_list
    
    #{"header": {"sec": sec, "nanosec": nanosec, ?("frame_id": frame_id)}}
    def parse_header(self, deserialized_data, omit_frame_id=True):
        header_dict = {
                    "header": {
                        "sec": deserialized_data.header.stamp.sec,
                        "nanosec": deserialized_data.header.stamp.nanosec,
                    }
                }
        if not omit_frame_id:
            header_dict["header"]["frame_id"] = deserialized_data.header.frame_id
        return header_dict
    
    #[{"header": header, "pose": {"position": [x1, y1, z1], "orientation": [x2, y2, z2, w]}}, ...]
    def parse_pose_stamped(self, deserialized_data, omit_frame_id=True):
        pose_stamped_list = []
        for pose_stamped in deserialized_data:
            pose_stamp = self.parse_header(pose_stamped, omit_frame_id=omit_frame_id)
            pose_stamp["pose"] = {
                                    "position": [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z], 
                                    "orientation": [pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, 
                                                    pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w]
                                }
            pose_stamped_list.append(pose_stamp)
        return pose_stamped_list
    
        #{"header": header, "poses": [pose_stamped, ...]}
    def parse_path(self, deserialized_data):
        #print(deserialized_data[0].header.stamp.sec)
        path_map = self.parse_header(deserialized_data[-1], omit_frame_id=False)
        #Take the pose list from last entry in path, because all poses are stored there (n*(n+1))/2 + c
        path_map["poses"] = self.parse_pose_stamped(deserialized_data[-1].poses)
        return path_map
    
    #[{"header": header, "height": height, "width": width, "encoding": encoding, "is_bigendian": is_bigendian}, ...]
    def pares_image(self, deserialized_data, offset, dir_name, file_name):
        bridge = CvBridge()
        image_list = []
        for i, image in enumerate(deserialized_data):
            image_dict = self.parse_header(image, omit_frame_id=False)
            image_dict["height"] = image.height
            image_dict["width"] = image.width
            image_dict["encoding"] = image.encoding
            image_dict["is_bigendian"] = image.is_bigendian
            image_list.append(image_dict)
            #Save raw rgb images 
            cv_img = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
            cv2.imwrite(os.path.join(dir_name, f"{i + offset}_{file_name}.png"), cv_img)
        return image_list
    
    #[{"header": header, "height": height, "width": width, "fields": [fields, ...], "is_bigendian": is_bigendian, "point_step": point_step, "is_dense": is_dense}, ...]
    def parse_pointcloud(self, deserialized_data, offset, dir_name, file_name):
        pointcloud_list = []
        for i, pointcloud in enumerate(deserialized_data):
            pointcloud_dict = self.parse_header(pointcloud, omit_frame_id=False)
            pointcloud_dict["height"] = pointcloud.height
            pointcloud_dict["width"] = pointcloud.width
            pointcloud_dict["fields"] = [{"name": pointfield.name, "offset": pointfield.offset, "datatype": pointfield.datatype, "count": pointfield.count} 
                                        for pointfield in pointcloud.fields]
            pointcloud_dict["is_bigendian"] = pointcloud.is_bigendian
            pointcloud_dict["point_step"] = pointcloud.point_step
            pointcloud_dict["row_step"] = pointcloud.row_step
            pointcloud_dict["is_dense"] = pointcloud.is_dense
            pointcloud_list.append(pointcloud_dict)
            #Save raw pointcloud data
            gen = np.array(point_cloud2.read_points_list(pointcloud, skip_nans=True))
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(gen[:, :-1])
            #Create RGB values from intensity
            intensity = np.log1p(gen[:, -1])
            intensity_normalized = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity))
            cmap = plt.get_cmap('RdYlGn')
            colors = cmap(intensity_normalized)[:, :3]
            pcd.colors = o3d.utility.Vector3dVector(colors)
            o3d.io.write_point_cloud(os.path.join(dir_name, f"{i + offset}_{file_name}.ply"), pcd)
        return pointcloud_list
    
    def export_to_json(self, json_obj, topic_name, bag_file):
        dir_name = os.path.dirname(bag_file)
        file_name = "-".join(((os.path.splitext(os.path.basename(bag_file))[0]) + topic_name).split("/")) + ".json"
        file = os.path.join(dir_name, file_name)
        with open(file, 'w', encoding='utf-8') as f:
            json.dump(json_obj, f)
    
def viz_path(path_map):
    fig = plt.figure()
    ax = Axes3D(fig)

    path_points = np.array([np.array(path["pose"]["position"]) for path in path_map["poses"]])
    #print(np.unique(path_points, axis=0))

    ax.plot3D(path_points[:, 0], path_points[:, 1], path_points[:, 2], 'green')
    ax.scatter3D(path_points[0][0], path_points[0][1], path_points[0][2], color='blue')
    ax.scatter3D(path_points[len(path_points) - 1][0], path_points[len(path_points) - 1][1], path_points[len(path_points) - 1][2], color='red')
    
    plt.show()

if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser("Bag parser")
    arg_parser.add_argument(
        "-b", "--bag", required=True, type=str, default=None, help="Path to ROS/ROS2 bag file that will be parsed"
    )
    arg_parser.add_argument(
        "-t", "--topic", required=True, type=str, default=None, help="Name of topic from which data should be extracted"
    )
    arg_parser.add_argument(
        "-n", "--name", type=str, default=None, help="If specified, name of the outgoing saved .png or .ply files will be changed"
    )
    arg_parser.add_argument(
        "-v", "--visualize_path", action="store_const", default=False, const=True, help="If specified, pathmap will be visualized"
    )
    arg_parser.add_argument(
        "-e", "--export_to_json", action="store_const", default=True, const=False, help="If specified, file will not be exported to json"
    )
    args, unknown_args = arg_parser.parse_known_args()
    print(args)
    if args.bag is None:
        raise ValueError(f"No bag file specified via --bag argument")
    
    if args.topic is None:
        raise ValueError(f"No topic name specified via --topic argument")
    
    parser = BagFileParser(args.bag)
    topic_type = parser.get_topic_type(args.topic)
    
    if (topic_type == "sensor_msgs/msg/Image" or topic_type == 'sensor_msgs/msg/PointCloud2'):
        print("Parsing batches...")
        parsed_data = parser.parse_batches(args.topic, args.name)

    else:
        print("Starting to deserialize...")
        deserialized_data = parser.get_messages(args.topic)
        print("Done with deserializing")
        print("Number of messages: " + str(len(deserialized_data)))
        print("Topic type: " + topic_type)
    
        print("Starting to parse...")
        if topic_type == 'nav_msgs/msg/Path':
            parsed_data = parser.parse_path(deserialized_data)
            if args.visualize_path:
                viz_path(parsed_data)
        elif topic_type == 'geometry_msgs/msg/PoseStamped':
            parsed_data = parser.parse_pose_stamped(deserialized_data, omit_frame_id=False)
        print("Done with parsing")

    if args.export_to_json:
        print("Exporting to json...")
        parser.export_to_json(parsed_data, args.topic, args.bag)
        print("Done with exporting")
    
    
    