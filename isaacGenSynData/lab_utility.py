#python
import json
import numpy as np
import open3d as o3d
from PIL import Image
from scipy.spatial.transform import Rotation

#isaac-core

#omniverse
from pxr import UsdGeom, Gf

# Load position and rotation data and interpolate points if specified
def load_data(file_path, interpolate, isLerp):
    points = []
    rotations = []
    with open(file_path, 'r', encoding='utf-8') as jsonf:
        path_data = json.load(jsonf)

    # Do linear interpolation between two points
    if isLerp:
        p0 = np.array(path_data["p0"])
        p1 = np.array(path_data["p1"])
        q0 = np.array(path_data["q0"])
        q1 = np.array(path_data["q1"])
        steps = path_data["steps"]
        points, rotations = interpolate_path(p0, p1, q0, q1, steps)

    # Interpolate missing points in given path data
    elif interpolate:
        print("> Interpolation starting...")
        with open(interpolate, 'r', encoding='utf-8') as jsonf:
            sim_data = json.load(jsonf)

        path_points = np.array([pose["pose"]["position"] for pose in path_data])
        #path_points[:, [1,2]] = path_points[:, [2,1]] #Swapping columns
        path_rotation = np.array([pose["pose"]["orientation"] for pose in path_data])
        path_time = [(pose["header"]["sec"] + (pose["header"]["nanosec"]/1e+9)) for pose in path_data]
        sim_data_time = [(image["header"]["sec"] + (image["header"]["nanosec"]/1e+9)) for image in sim_data]

        time_intervals = zip(path_time, path_time[1:])
        point_intervals = list(zip(path_points, path_points[1:]))
        for i, (start_t, end_t) in enumerate(time_intervals):
            for image_t in sim_data_time:
                if start_t == image_t:
                    points.append(path_points[i])
                    rotations.append(path_rotation[i])
                    continue
                if start_t < image_t < end_t:
                    p0 = point_intervals[i][0]
                    p1 = point_intervals[i][1]
                    d = (image_t - start_t)/(end_t - start_t)
                    pd = (1-d)*p0 + d*p1
                    points.append(pd)
                    rotations.append(path_rotation[i])

        points.append(path_points[-1])
        rotations.append(path_rotation[-1])
        print("> Interpolation ended")

    # Save path without interpolating missing positions
    else:
        points = np.array([pose["pose"]["position"] for pose in path_data])
        #self._points[:, [1,2]] = self._points[:, [2,1]] #Swapping columns
        rotations = np.array([pose["pose"]["orientation"] for pose in path_data])
    
    # Change coordinate system to origin position
    # Extract the first position and rotation
    origin_position = points[0]
    origin_rotation = rotations[0]
    
    # Translate the positions
    translated_positions = points - origin_position
    
    # Rotate the positions
    origin_rotation_conjugate = Rotation.from_quat(origin_rotation).inv().as_quat() # Get inverse of new origin orientation
    rotated_positions = Rotation.from_quat(origin_rotation_conjugate).apply(translated_positions)
    
    # Update the rotations by multyplying all orientations by the origin inverse
    updated_rotations = np.array([(Rotation.from_quat(q) * Rotation.from_quat(origin_rotation_conjugate)).as_quat() for q in rotations])
    #updated_rotations = np.array([r.as_quat() for r in updated_rotations])

    points = rotated_positions
    rotations = updated_rotations

    print(f"Total of {len(points)} points in path")
    return points, rotations

# Save rgb image to file
def save_rgb(rgb_data, file_name):
    rgb_image_data = np.frombuffer(rgb_data, dtype=np.uint8).reshape(512, 896, 4)
    rgb_img = Image.fromarray(rgb_image_data, "RGBA")
    rgb_img.save(file_name + ".png")

# Save pcd to file
def save_pcd(pcd_data, file_name):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_data)
    o3d.io.write_point_cloud(file_name, pcd)

# SLERP implementation
def slerp(q1, q2, t):
    # Normalize the input quaternions
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    dot = np.dot(q1, q2)
    # Ensure the shortest path by reversing one quaternion if necessary
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    # Calculate the interpolation angle
    omega = np.arccos(dot)
    sin_omega = np.sin(omega)
    # Perform SLERP
    result = (np.sin((1 - t) * omega) * q1 + np.sin(t * omega) * q2) / sin_omega
    
    return result

# Linear interpolation of a linear path
def interpolate_path(p0, p1, q0, q1, steps):
    points = []
    rotations = []
    for i in range(steps+1):
        t = i/steps
        new_pos = (1-t)*p0 + t*p1
        new_rot = slerp(q0, q1, t)
        #print(new_rot)
        points.append(new_pos)
        rotations.append(new_rot)
    return points, rotations

# Rotate and tranlate camera matrix
def rotate_camera(stage, camera_path, x):
    # Get the camera prim
    camera_prim = stage.GetPrimAtPath(camera_path)
    if not camera_prim.IsValid():
        print(f"Camera prim not found at path: {camera_path}")
        return
    
    xformable = UsdGeom.Xformable(camera_prim)
    xformable.ClearXformOpOrder()
    transform = xformable.AddTransformOp()

    #XZY -> XYZ  
    mat = Gf.Matrix4d(
        1, 0, 0, 0,
        0, 0, 1, 0,
        0, -1, 0, 0,
        x, 0, 0, 1
    )
    #mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(1,0,1), -90.0))
    transform.Set(mat)

# Set pose of prim in stage
def set_pose(stage, prim_path, position=None, orientation=None):
    prim = stage.GetPrimAtPath(str(prim_path))
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    transform = xform.AddTransformOp()
    mat = Gf.Matrix4d()
    mat2 = Gf.Matrix4d()
    mat2.SetIdentity()
    if position is not None:
        mat.SetTranslateOnly(Gf.Vec3d(*position))
    if orientation is not None:
        mat.SetRotateOnly(Gf.Rotation(Gf.Quaternion(orientation[-1], Gf.Vec3d(*orientation[:-1]))))
        mat2.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0,0,1), -90.0)) # Rotation correction
    # Only manipluate the rotation and not the translation
    rot_mat = mat.ExtractRotationMatrix()
    rot_mat2 = mat2.ExtractRotationMatrix()
    rot_mat *= rot_mat2
    mat.SetRotateOnly(rot_mat)
    transform.Set(mat)

# Create camera in stage with specified parameters
def create_camera(stage, parent, name, k, width, height):
    left_camera = UsdGeom.Camera.Define(stage, parent + name)
    left_camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
    left_camera.CreateFocalLengthAttr().Set(k[0,0])
    left_camera.CreateHorizontalApertureAttr().Set(width)
    left_camera.CreateVerticalApertureAttr().Set(height)
    left_camera.CreateClippingRangeAttr().Set((0.1,100000))