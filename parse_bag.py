import os
import json
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from mpl_toolkits.mplot3d import axes3d, Axes3D
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

#ROS2 has to be installed on the systen, follow installation steps: https://docs.ros.org/en/foxy/Installation.html
#Setup the ROS2 enviroment first before running the script via: . ~/ros2_foxy/install/local_setup.bash

#bag_file = 'zed_kuba_1_0.db3'
#bag_file = '../lidar_kuba_1/lidar_kuba_1_0.db3'
bag_file = '../lidar_kuba_2/lidar_kuba_2_0.db3'
#bag_file = '../lidar_kuba_3/lidar_kuba_3_0.db3'
topic_name = '/zed2/zed_node/path_map'
#topic_name = '/zed2/zed_node/pose'
#topic_name = '/zed2/zed_node/left_raw/image_raw_color'
#topic_name = '/rslidar_points'
visualize_path = True
export_to_json = False
db_offset = 451 #Only for messages of type Image, fetching all images at once causes script to crash

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

        #If message type is Image, deserialize not all at once
        if (self.topic_type[topic_name] == "sensor_msgs/msg/Image"):
            msg_list = []
            rows_empty = False
            i = 0
            while not rows_empty:
                rows = self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id} LIMIT {db_offset} OFFSET {i*db_offset + 902}").fetchall()
                if len(rows) == 0:
                    rows_empty = True
                else: 
                    msg_list += [deserialize_message(data, self.topic_msg_message[topic_name]) for timestamp, data in rows]
                    i += 1
                if i*db_offset >= 900:
                    return msg_list
            return msg_list
        
        #Get rows from the db
        rows = self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id}").fetchall()
        #Deserialise all messages
        #return return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]
        return [deserialize_message(data, self.topic_msg_message[topic_name]) for timestamp, data in rows]
    
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
    def pares_image(self, deserialized_data):
        image_list = []
        for image in deserialized_data:
            image_dict = self.parse_header(image, omit_frame_id=False)
            image_dict["height"] = image.height
            image_dict["width"] = image.width
            image_dict["encoding"] = image.encoding
            image_dict["is_bigendian"] = image.is_bigendian
            image_list.append(image_dict)
        return image_list
    
    #[{"header": header, "height": height, "width": width, "fields": [fields, ...], "is_bigendian": is_bigendian, "point_step": point_step, "is_dense": is_dense}, ...]
    def parse_pointcloud(self, deserialized_data):
        pointcloud_list = []
        for pointcloud in deserialized_data:
            pointcloud_dict = self.parse_header(pointcloud, omit_frame_id=False)
            pointcloud_dict["height"] = pointcloud.height
            pointcloud_dict["width"] = pointcloud.width
            pointcloud_dict["fields"] = [{"name": pointfield.name, "offset": pointfield.offset, "datatype": pointfield.datatype, "count": pointfield.count} 
                                        for pointfield in pointcloud.fields]
            pointcloud_dict["is_bigendian"] = pointcloud.is_bigendian
            pointcloud_dict["point_step"] = pointcloud.point_step
            pointcloud_dict["row_step"] = pointcloud.row_step
            #data
            pointcloud_dict["is_dense"] = pointcloud.is_dense
            pointcloud_list.append(pointcloud_dict)
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
    parser = BagFileParser(bag_file)

    print("Starting to deserialize...")
    deserialized_data = parser.get_messages(topic_name)
    print("Done with deserializing")
    print("Number of messages: " + str(len(deserialized_data)))
    
    topic_type = parser.get_topic_type(topic_name)
    print("Topic type: " + topic_type)

    print("Starting to parse...")
    if topic_type == 'nav_msgs/msg/Path':
        parsed_data = parser.parse_path(deserialized_data)
        if visualize_path:
            viz_path(parsed_data)
    elif topic_type == 'geometry_msgs/msg/PoseStamped':
        parsed_data = parser.parse_pose_stamped(deserialized_data, omit_frame_id=False)
    elif topic_type == 'sensor_msgs/msg/Image':
        parsed_data = parser.pares_image(deserialized_data)
    elif topic_type == 'sensor_msgs/msg/PointCloud2':
        parsed_data = parser.parse_pointcloud(deserialized_data)
    print("Done with parsing")

    if export_to_json:
        print("Exporting to json...")
        parser.export_to_json(parsed_data, topic_name, bag_file)
        print("Done with exporting")
    
    
    