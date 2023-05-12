from omni.isaac.kit import SimulationApp
config = {
     'window_width': "1280",
     'window_height': "720",
     'headless': False,
}
simulation_app = SimulationApp(config)

#python
import os
import pptk
import copy
import argparse
import numpy as np
import open3d as o3d
from threading import Thread
from lab_utility import create_camera, rotate_camera, set_pose, load_data, save_rgb

#isaac-core
from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage, update_stage
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.range_sensor import _range_sensor 

#omniverse
import omni.kit
import omni.usd
import omni.replicator.core as rep
from omni.replicator.core import AnnotatorRegistry
from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf, Semantics, UsdPhysics

class LabRun():
    def __init__(self, file_path, interpolate = None, isLidar = False, isStereo = False) -> None:
        self._world = None
        self._stage = None
        self._current_tasks = None
        self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
        self._isLidar = isLidar
        self._isStereo = isStereo
        self._i = 0
        self._camera_height = 512
        self._camera_width = 896
        self._camera_k = np.array([[364.2601318359375, 0.0, 443.48822021484375],
                                   [0.0, 370.04205322265625, 252.1682891845703],
                                   [0.0, 0.0, 1.0]])
        self._pointclouds = [] # Pointclouds generated are stored here
        self._trans_mats = [] # Homogenous 4x4 transform matrices for each time step

        self._points, self._rotations = load_data(file_path, interpolate)
    
    def load_world(self):
        print("> Load simulation...")
        if World.instance() is None:
            create_new_stage()
            self._world = World(**self._world_settings)
            #await self._world.initialize_simulation_context_async()
            self.setup_scene()
        else:
            self._world = World.instance()
        self._current_tasks = self._world.get_current_tasks()
        self._world.reset()
        self._world.pause()
        self.setup_post_load()
        if len(self._current_tasks) > 0:
            self._world.add_physics_callback("tasks_step", self._world.step_async)
        print("> Loading done")

    def reset(self):
        print("> Resetting...")
        if self._world.is_tasks_scene_built() and len(self._current_tasks) > 0:
            self._world.remove_physics_callback("tasks_step")
        self._world.play()
        update_stage()
        #await self.setup_pre_reset()
        self._world.reset()
        self._world.pause()
        self.setup_post_reset()
        if self._world.is_tasks_scene_built() and len(self._current_tasks) > 0:
            self._world.add_physics_callback("tasks_step", self._world.step_async)
        print("> Resetting done")

    def setup_scene(self):
        print("> Setting up scene...")
        #Get stage of current scene
        self._stage = omni.usd.get_context().get_stage()

        #Add groundplane
        self._world.scene.add_default_ground_plane()
        prim_groundplane = self._stage.GetPrimAtPath("/World/defaultGroundPlane")
        self.groundplane = UsdGeom.Xformable(prim_groundplane)

        #Add acopos object to the scene and applay collision and semantics API
        #acopos_asset_path = "omniverse://localhost/Projects/acoposobj_scaled/acoposobj_scaled.usd" isaacGenSynData
        acopos_asset_path = "./isaacGenSynData/acoposobj_scaled.usd"
        self.acoposObj = add_reference_to_stage(usd_path=acopos_asset_path, prim_path="/World/Acopos")
        set_pose(self._stage, "/World/Acopos", [3, 1, 0.27], [0.5, -0.5, -0.5, 0.5])
        collisionAPI = UsdPhysics.CollisionAPI.Apply(self.acoposObj)
        sem = Semantics.SemanticsAPI.Apply(self.acoposObj, "Semantics")
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set("class")
        sem.GetSemanticDataAttr().Set("Acopos")

        #Add robot to the scene
        self.robotObj = UsdGeom.Xform.Define(self._stage, "/World/Robot")
        base = UsdGeom.Cylinder.Define(self._stage, "/World/Robot/Base")
        base.CreateRadiusAttr(0.4)
        base.CreateHeightAttr(0.01)
        mast = UsdGeom.Cylinder.Define(self._stage, "/World/Robot/Mast")
        mast.CreateRadiusAttr(0.02)
        mast.CreateHeightAttr(1)
        mast.AddTranslateOp().Set((0, 0, 0.5))

        if self._isStereo:
            print("> Setting up Stereo Camera...")

            #zed2_asset_path = "omniverse://localhost/Projects/ZED2_scaled/ZED2_scaled.usd"
            zed2_asset_path = "./isaacGenSynData/ZED2_scaled.usd"
            self.zed2Obj = add_reference_to_stage(usd_path=zed2_asset_path, prim_path="/World/Robot/ZED2")
            set_pose(self._stage, "/World/Robot/ZED2", [0, 0, 0.7])

            #Create left and right stereo cameras
            create_camera(self._stage, "/World/Robot/ZED2", "/CameraLeft", self._camera_k, self._camera_width, self._camera_height)
            create_camera(self._stage, "/World/Robot/ZED2", "/CameraRight", self._camera_k, self._camera_width, self._camera_height)
            rotate_camera(self._stage, "/World/Robot/ZED2/CameraLeft", 0.06)
            rotate_camera(self._stage, "/World/Robot/ZED2/CameraRight", -0.06)
            
            #Create render products
            rp0 = rep.create.render_product("/World/Robot/ZED2/CameraLeft", resolution=(self._camera_width, self._camera_height))
            rp1 = rep.create.render_product("/World/Robot/ZED2/CameraRight", resolution=(self._camera_width, self._camera_height))

            #Access the data through annotators
            self.rgb_annotators = []
            for rp in [rp0, rp1]:
                rgb = AnnotatorRegistry.get_annotator("rgb")
                rgb.attach([rp])
                self.rgb_annotators.append(rgb)

            #Create annotator output directory
            file_path = os.path.join(os.getcwd(), "/isaacGenSynData/_out_annot", "")
            print(f"Writing annotator data to {file_path}")
            self.dir_name_img = os.path.dirname(file_path)
            os.makedirs(self.dir_name_img, exist_ok=True)

        if self._isLidar:
            print("> Setting up Lidar Sensor...")

            self.lidarObj = UsdGeom.Cylinder.Define(self._stage, "/World/Robot/LidarObj")
            self.lidarObj.CreateRadiusAttr(0.05)
            self.lidarObj.CreateHeightAttr(0.1)
            self.lidarObj.AddTranslateOp().Set((0, 0, 1.05))

            self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
            result, lidarPrim = omni.kit.commands.execute(
                "RangeSensorCreateLidar",
                path="/Lidar",
                parent="/World/Robot/LidarObj",
                min_range=0.4,
                max_range=150.0,
                draw_points=True,
                draw_lines=False,
                horizontal_fov=360.0,
                vertical_fov=70.0,
                horizontal_resolution=0.4,
                vertical_resolution=1.33,
                rotation_rate=0.0,
                high_lod=True,
                yaw_offset=0.0,
                enable_semantics=True
            )
            UsdGeom.XformCommonAPI(lidarPrim).SetTranslate((0.0, 0.0, 0.0))

            #Create pointcloud output directory
            file_path = os.path.join(os.getcwd(), "/isaacGenSynData/_out_pcd", "")
            print(f"Writing pointcloud data to {file_path}")
            self.dir_name_pcd = os.path.dirname(file_path)
            os.makedirs(self.dir_name_pcd, exist_ok=True)

        print("> Setup done")
    
    def setup_post_load(self):
        print("> Setup post load...")
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        #self._world.play()
        print("> Setup post load done")

    def setup_post_reset(self):
        print("> Setup post reset...")
        self._i = 0
        set_pose(self._stage, "/World/Robot", [0, 0, 0], [0, 0, 0, 1])
        if self._isLidar:
            self._pointclouds = []
        #self._world.play()
        print("> Setup post reset done")

    def get_pointcloud(self):
        self._world.pause()
        pointcloud = self.lidarInterface.get_point_cloud_data("/World/Robot/LidarObj/Lidar")
        #semantics = self.lidarInterface.get_semantic_data("/World/ZED2/Lidar")
        pointcloud = pointcloud.reshape(47700, 3)
        self._pointclouds.append(pointcloud)

        #Get transform 
        xformable = UsdGeom.Xformable(self._stage.GetPrimAtPath("/World/Robot"))
        transform = xformable.GetLocalTransformation()
        mat = np.array(transform)
        T = np.empty((4, 4))
        T[:3, :3] = mat[:3, :3]
        T[:3, 3] = mat[3, :3]
        T[3, :] = [0, 0, 0, 1]
        self._trans_mats.append(T)

        self._world.play()

    def get_image(self):
        #rep.orchestrator.step() self.rgb_annotators
        for j, rgb_annot in enumerate(self.rgb_annotators):
            MyThread = Thread(target=save_rgb, args=(rgb_annot.get_data(), f"{self.dir_name_img}/rp{j}_step_{self._world.current_time_step_index}"))
            MyThread.start()

    def physics_step(self, step_size):
        if self._i == 250:
            print("250")
        if self._i == 500:
            print("500")
        if self._i < len(self._points):
            #self._points[self._i][2] = 0
            if self._isLidar:
                set_pose(self._stage, "/World/Robot", self._points[self._i])
                self.get_pointcloud()

            elif self._isStereo:
                set_pose(self._stage, "/World/Robot", self._points[self._i], self._rotations[self._i])
                #with self.stereo_camera_pair:
                #    rep.modify.pose(position=self._points[self._i], rotation=self.euler_from_quaternion(*self._rotations[self._i]))
                self.get_image()
        else:
            self._world.pause()
        self._i += 1
        #self._world.pause()

def align_pointclouds(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)

    source = np.asarray(source_temp.points)
    target = np.asarray(target_temp.points)
    pointcloud = np.concatenate([source, target])
    return pointcloud

def get_aligned_pointcloud(pointclouds, trans_mats, steps=1, icp=False):
    print(f"Aligning a total of {len(pointclouds)} pointclouds")
    threshold = 0.2
    source = []
    target = []
    for i in range(0, len(pointclouds), steps):
        if i+steps < len(pointclouds):
            if i == 0:
                continue
            print(f"Aliging pointcloud {i+steps}")
            if i == steps:
                source = pointclouds[i+steps]
                target = pointclouds[i]
                pcd_s = o3d.geometry.PointCloud()
                pcd_s.points = o3d.utility.Vector3dVector(source)
                pcd_t = o3d.geometry.PointCloud()
                pcd_t.points = o3d.utility.Vector3dVector(target)

                trans_init = trans_mats[i+steps]

                if icp:
                    reg_p2p = o3d.registration.registration_icp(
                        pcd_s, pcd_t, threshold, trans_init,
                        o3d.registration.TransformationEstimationPointToPoint())

                    target = align_pointclouds(pcd_s, pcd_t, reg_p2p.transformation)
                else:
                    target = align_pointclouds(pcd_s, pcd_t, trans_init)
            else:
                source = pointclouds[i+steps]
                pcd_s = o3d.geometry.PointCloud()
                pcd_s.points = o3d.utility.Vector3dVector(source)
                pcd_t = o3d.geometry.PointCloud()
                pcd_t.points = o3d.utility.Vector3dVector(target)

                trans_init = trans_mats[i+steps]

                if icp:
                    reg_p2p = o3d.registration.registration_icp(
                        pcd_s, pcd_t, threshold, trans_init,
                        o3d.registration.TransformationEstimationPointToPoint())

                    target = align_pointclouds(pcd_s, pcd_t, reg_p2p.transformation)
                else:
                    target = align_pointclouds(pcd_s, pcd_t, trans_init)
    print("Done")
    return target

def main():
    parser = argparse.ArgumentParser("Virtual lab run")
    parser.add_argument(
        "-p", "--path", required=True, type=str, default=None, help=".json file that specifies the pathmap"
    )
    parser.add_argument(
        "-i", "--interpolate", type=str, default=None, help="If provided a .json file of recorded image or pointcloud data with timestamps then interpolate missing poses"
    )
    parser.add_argument(
        "-l", "--lidar", action="store_const", default=False, const=True, help="If specified, a lidar scan will be done"
    )
    parser.add_argument(
        "-s", "--stereo", action="store_const", default=False, const=True, help="If specified, a scan with stereo vision will be done"
    )
    args, unknown_args = parser.parse_known_args()
    print(args)
    if args.path is None:
        raise ValueError(f"No path specified via --path argument")
    
    lab_run = LabRun(args.path, args.interpolate, isLidar=args.lidar, isStereo=args.stereo)
    lab_run.load_world()

    while simulation_app.is_running():
        #Take a step in the simulation
        lab_run._world.step(render=True)
        if lab_run._world.is_playing():
            #Reset the world
            if lab_run._world.current_time_step_index == 0:
                lab_run.reset()

    # cleanup
    simulation_app.close()
    
    # Save pointcloud data
    if args.lidar:
    
        pointcloud = get_aligned_pointcloud(lab_run._pointclouds, lab_run._trans_mats, steps=100)
        x, y, z = pointcloud[:, 0], pointcloud[:, 1], pointcloud[:, 2]
        mask = (x > -10) & (x < 10) & (y > -10) & (y < 10) & (z > -1)
        pointcloud = pointcloud[mask]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud)
        o3d.io.write_point_cloud(f"{lab_run.dir_name_pcd}/registered_synthetic_pcd.ply", pcd)

        #v = pptk.viewer(pointcloud)

if __name__ == "__main__":
    main()