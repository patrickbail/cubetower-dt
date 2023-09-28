#python
import cv2
import json
import math
import numpy as np
import open3d as o3d
from PIL import Image
from scipy.optimize import curve_fit
from scipy.spatial.transform import Rotation

#isaac-core
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core.utils.extensions import enable_extension 
from omni.isaac.sensor import Camera

#omniverse
import omni.graph.core as og
from pxr import UsdGeom, Gf

# Load position and rotation data and interpolate points if specified
# TODO when interpolating remember indicices of timestamps that have no interpolated pose
def load_data(file_path, file_sensor, isLerp):
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

    else:
        points = np.array([pose["pose"]["position"] for pose in path_data])
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

        # Interpolate missing points in given path data
        if file_sensor:
            print("> Interpolation starting...")
            with open(file_sensor, 'r', encoding='utf-8') as jsonf:
                sensor_data = json.load(jsonf)

            path_points = []
            path_rotations = []
            path_time = [(pose["header"]["sec"] + (pose["header"]["nanosec"]/1e+9)) for pose in path_data]
            sensor_time = [(image["header"]["sec"] + (image["header"]["nanosec"]/1e+9)) for image in sensor_data]

            time_intervals = zip(path_time, path_time[1:])
            point_intervals = list(zip(points, points[1:]))
            rotation_intervals = list(zip(rotations, rotations[1:]))
            for i, (start_t, end_t) in enumerate(time_intervals):
                for sensor_t in sensor_time:
                    # Check if last timestamp interval
                    if i == len(path_time) - 1:
                        if end_t == sensor_t:
                            path_points.append(points[-1])
                            path_rotations.append(rotations[-1])
                            continue
                    # Timestamps match, no interpolation needed
                    if start_t == sensor_t:
                        path_points.append(points[i])
                        path_rotations.append(rotations[i])
                        continue
                    # Interpolate pose for missing match
                    if start_t < sensor_t < end_t:
                        # LERP
                        p0 = point_intervals[i][0]
                        p1 = point_intervals[i][1]
                        d = (sensor_t - start_t)/(end_t - start_t)
                        pd = (1-d)*p0 + d*p1
                        path_points.append(pd)
                        # SLERP
                        q0 = rotation_intervals[i][0]
                        q1 = rotation_intervals[i][1]
                        qd = slerp(q0, q1, d)
                        #path_rotations.append(rotations[i])
                        path_rotations.append(qd)

            points = path_points
            rotations = path_rotations
            print("> Interpolation ended")

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

    # If the quaternions are very close, just linearly interpolate
    if abs(sin_omega) < 1e-6:
        #print("Linear")
        return (1-t)*q1 + t*q2

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

# Set pose of prim in stage
def set_pose(stage, prim_path, position=None, orientation=None):
    prim = stage.GetPrimAtPath(str(prim_path))
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    transform = xform.AddTransformOp()
    mat = Gf.Matrix4d()
    if position is not None:
        mat.SetTranslateOnly(Gf.Vec3d(*position))
    if orientation is not None:
        mat.SetRotateOnly(Gf.Rotation(Gf.Quaternion(orientation[-1], Gf.Vec3d(*orientation[:-1]))))
    transform.Set(mat)

'''
def DistortPoint(x, y, camera_matrix, distortion_coefficients):
    ((fx,_,cx),(_,fy,cy),(_,_,_)) = camera_matrix
    pt_x, pt_y, pt_z  = (x-cx)/fx, (y-cy)/fy, np.full(x.shape, 1.0)
    points3d = np.stack((pt_x, pt_y, pt_z), axis = -1)
    rvecs, tvecs = np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0])
    cameraMatrix, distCoeffs = np.array(camera_matrix), np.array(distortion_coefficients)
    points, jac = cv2.projectPoints(points3d, rvecs, tvecs, cameraMatrix, distCoeffs)
    return np.array([points[:,0,0], points[:,0,1]])

def Theta(x, y, camera_matrix):
   ((fx,_,cx),(_,fy,cy),(_,_,_)) = camera_matrix
   pt_x, pt_y, pt_z  = (x-cx)/fx, (y-cy)/fy, 1.0
   r2 = pt_x * pt_x + pt_y * pt_y
   theta = np.arctan2(np.sqrt(r2), 1.0)
   return theta

def poly4_00(theta, b, c, d, e):
    return b*theta + c*np.power(theta,2) + d*np.power(theta,3) + e*np.power(theta, 4)
'''

# Create camera in stage with specified parameters
# TODO needs actual parameters
def create_camera(stage, parent, name, camera_matrix, width, height, position, stereoRole="mono"):
    focal_length = 2.12 # 2.12 mm according to the ZED 2 specs
    Pixel_size = 2      # in µm
    vert_aperture = (height * focal_length) / camera_matrix[1,1]
    horiz_aperture = (width * focal_length) / camera_matrix[0,0]
    camera = UsdGeom.Camera.Define(stage, parent + name)
    camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
    #camera.CreateFocalLengthAttr().Set(k[0,0])
    camera.CreateFocalLengthAttr().Set(focal_length)
    #camera.CreateFocalLengthAttr().Set((camera_matrix[0,0] * Pixel_size * 1e-3 + camera_matrix[1,1] * Pixel_size * 1e-3) / 2)
    #camera.CreateHorizontalApertureAttr().Set(width)
    camera.CreateHorizontalApertureAttr().Set(horiz_aperture)
    #camera.CreateHorizontalApertureAttr().Set(Pixel_size * 1e-3 * width)
    #camera.CreateVerticalApertureAttr().Set(height)
    camera.CreateVerticalApertureAttr().Set(vert_aperture)
    #camera.CreateVerticalApertureAttr().Set(Pixel_size * 1e-3 * height)
    #camera.CreateFStopAttr(1.8)
    camera.CreateClippingRangeAttr().Set((0.1,1000000))
    camera.AddTranslateOp().Set(Gf.Vec3d(*position))
    camera.CreateStereoRoleAttr(stereoRole)
    # Position camera
    #xformable = UsdGeom.Xformable(camera)
    # Rotate camera and make it look down the +X axis with +Z upwards since cameras look down the -Z axis with +Y upwards
    #camera.AddRotateXZYOp().Set(Gf.Vec3d(0, -90, -90))
    '''
    # ROS unsupported distortion model
    # https://forums.developer.nvidia.com/t/how-to-insert-camera-intrinsic-matrix-and-distortion-coefficients-for-the-camera-other-than-fisheyepolynomial/252014/4
    Pixel_size = 2 # in µm
    distortion_coefficients = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ((fx,_,cx),(_,fy,cy),(_,_,_)) = camera_matrix

    # Fit the fTheta model for the points on the diagonal.
    X = np.linspace(cx,width, width)
    Y = np.linspace(cy,height, width)
    theta = Theta(X, Y, camera_matrix)
    r = np.linalg.norm(DistortPoint(X, Y, camera_matrix, distortion_coefficients) - np.array([[cx], [cy]]), axis=0)
    order4_00_coeffs, _ = curve_fit(poly4_00, r, theta)

    # The coefficient 'a' of the Ftheta model is set to zero, so that angle 0 is at pixel distance (aka radius) 0
    Ftheta_A = [0.0] + list(order4_00_coeffs)
    Dfov = np.rad2deg(2*poly4_00(np.linalg.norm([height/2,width/2]) , *order4_00_coeffs))

    camera = UsdGeom.Camera.Define(stage, parent + name)
    camera.AddTranslateOp().Set(Gf.Vec3d(*position))

    camera = Camera(parent + name)
    camera.set_resolution((width, height))
    camera.set_focal_length((fx * Pixel_size * 1e-3 + fy * Pixel_size * 1e-3) / 2)  # in mm
    camera.set_horizontal_aperture(Pixel_size * 1e-3 * width)                       # in mm
    camera.set_vertical_aperture(Pixel_size * 1e-3 * height)                        # in mm
    camera.set_projection_mode("perspective")
    camera.set_projection_type("fisheyePolynomial")
    camera.set_stereo_role(stereoRole)
    camera.set_fisheye_polynomial_properties(
        nominal_width=width, nominal_height=height,
        optical_centre_x=cx, optical_centre_y=cy,
        max_fov = Dfov, polynomial = Ftheta_A)
    camera.set_clipping_range(near_distance=0.0, far_distance=1000000.0)
    '''