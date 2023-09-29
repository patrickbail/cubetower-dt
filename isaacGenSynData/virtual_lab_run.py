from omni.isaac.kit import SimulationApp
config = {
    "headless": True,
    "active_gpu": None,
    "physics_gpu": 0,
    "multi_gpu": True,
    "sync_loads": True,
    "width": 896,
    "height": 512,
    "window_width": 1920,
    "window_height": 1080,
    "display_options": 3094,
    "subdiv_refinement_level": 0,
    "renderer": "RayTracedLighting",  # Can also be PathTracing
    "anti_aliasing": 3, # DLSS
    "samples_per_pixel_per_frame": 64,
    "denoiser": True,
    "max_bounces": 4,
    "max_specular_transmission_bounces": 6,
    "max_volume_bounces": 4,
    "open_usd": None,
    "livesync_usd": None,
}
simulation_app = SimulationApp(config)

# For building own ROS Docker container with ZED SDK and wrapper node
# ZED SDK on ROS 2 docker container
# wget https://download.stereolabs.com/zedsdk/4.0/cu118/ubuntu20 -O ZED_SDK_Ubuntu22_cuda11.8_v4.0.0.zstd.run
# chmod +x ZED_SDK_Ubuntu22_cuda11.8_v4.0.0.zstd.run
# ./ZED_SDK_Ubuntu22_cuda11.8_v4.0.0.zstd.run -- silent
# Rest on here https://github.com/stereolabs/zed-ros2-wrapper
# Or simply follow this guide, for prebuild docker:
# https://github.com/stereolabs/zed-ros2-wrapper/tree/master/docker

#python
import os
import argparse
import numpy as np
from threading import Thread
from lab_utility import create_camera, set_pose, load_data, save_rgb, save_pcd

#isaac-core
from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage, update_stage
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension 
from omni.isaac.core.utils.render_product import create_hydra_texture
from omni.isaac.range_sensor import _range_sensor

#omniverse
import omni.kit
import omni.usd
import omni.graph.core as og
import omni.replicator.core as rep
from omni.replicator.core import AnnotatorRegistry
from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf

class LabRun():
    def __init__(self, file_path, file_sensor = None, isLidar = False, isStereo = False, isLerp = False, earlyStopping = -1) -> None:
        self.DEBUG_CAMERA = False
        self.DEBUG_LIDAR = False
        self._world = None
        self._stage = None
        self._current_tasks = None
        self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
        self._isLidar = isLidar
        self._isStereo = isStereo
        self._isLerp = isLerp
        self._earlyStopping = earlyStopping
        self._i = 0 # Time step index
        self._camera_height = 512
        self._camera_width = 896
        self._camera_k = np.array([[364.2601318359375, 0.0, 443.48822021484375],
                                   [0.0, 370.04205322265625, 252.1682891845703],
                                   [0.0, 0.0, 1.0]])
        self._pointclouds = [] # Pointclouds generated are stored here
        self._trans_mats = [] # Homogenous 4x4 transform matrices for each time step
        #self._origin_points = []

        self._points, self._rotations = load_data(file_path, file_sensor, isLerp)
    
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

        add_reference_to_stage(
            usd_path="./Isaac-Sim-Playground/Assets/USD-Files/cube_setup5.usd", prim_path="/World"
        )

        #Add robot to the scene
        add_reference_to_stage(
            usd_path="./Isaac-Sim-Playground/Assets/USD-Files/robot.usd", prim_path="/World/robot"
        )

        if self._isStereo:
            print("> Setting up Stereo Camera...")

            # Add refrence of ZED 2 camera model 
            #zed2_asset_path = "omniverse://localhost/Projects/ZED2_scaled/ZED2_scaled.usd" # Nucleus version
            zed2_asset_path = "./Isaac-Sim-Playground/Assets/USD-Files/ZED2_scaled.usd" # Local version
            add_reference_to_stage(usd_path=zed2_asset_path, prim_path="/World/robot/Robot/ZED2")
            # Rotate ZED 2 camera, make it look down the +X axis with +Z upwards since cameras look down the -Z axis with +Y upwards
            zed_prim = self._stage.GetPrimAtPath("/World/robot/Robot/ZED2")            
            xformable = UsdGeom.Xformable(zed_prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp().Set(Gf.Vec3d([0.24, 0, 0.7]))
            xformable.AddRotateXZYOp().Set(Gf.Vec3d(0, -90, -90))

            #Create left and right stereo cameras
            create_camera(self._stage, "/World/robot/Robot/ZED2", "/CameraLeft", self._camera_k, 
                          self._camera_width, self._camera_height, [0.06, 0, 0], "left")
            create_camera(self._stage, "/World/robot/Robot/ZED2", "/CameraRight", self._camera_k, 
                          self._camera_width, self._camera_height, [-0.06, 0, 0], "right")
            create_camera(self._stage, "/World/robot/Robot/ZED2", "/DepthCamera", self._camera_k, 
                          self._camera_width, self._camera_height, [0., 0, 0])
            
            #Create render products
            rp0 = rep.create.render_product("/World/robot/Robot/ZED2/CameraLeft", resolution=(self._camera_width, self._camera_height))
            rp1 = rep.create.render_product("/World/robot/Robot/ZED2/CameraRight", resolution=(self._camera_width, self._camera_height))
            rp2 = rep.create.render_product("/World/robot/Robot/ZED2/DepthCamera", resolution=(self._camera_width, self._camera_height))

            # Create ROS 2 Camera Info and Image publisher pipeline in the post process graph
            writer = rep.writers.get("ROS2PublishCameraInfo")
            writer.initialize(topicName="left/camera_info")
            writer.attach([rp0])
            writer = rep.writers.get("LdrColorSD" + "ROS2PublishImage")
            writer.initialize(topicName="left/image_rect")
            writer.attach([rp0])

            writer = rep.writers.get("ROS2PublishCameraInfo")
            writer.initialize(topicName="right/camera_info", stereoOffset=[-43.717472076416016, 0.0])
            writer.attach([rp1])
            writer = rep.writers.get("LdrColorSD" + "ROS2PublishImage")
            writer.initialize(topicName="right/image_rect")
            writer.attach([rp1])

            writer = rep.writers.get("DistanceToImagePlaneSD" + "ROS2PublishImage")
            writer.initialize(topicName="depth")
            writer.attach([rp2])

            #Access the data through annotators
            self.rgb_annotators = []
            for i, rp in enumerate([rp0, rp1, rp2]):
                if i == 2:
                    annot = AnnotatorRegistry.get_annotator("distance_to_image_plane")
                else:
                    annot = AnnotatorRegistry.get_annotator("rgb")
                annot.attach([rp])
                self.rgb_annotators.append(annot)

            #Create annotator output directory
            file_path = os.path.join(os.getcwd(), "Isaac-Sim-Playground", "isaacGenSynData" , "_out_image", "")
            print(f"Writing annotator data to {file_path}")
            self.dir_name_img = os.path.dirname(file_path)
            os.makedirs(self.dir_name_img, exist_ok=True)

        if self._isLidar:
            
            print("> Setting up Lidar Sensor...")

            #self.lidarObj = UsdGeom.Cylinder.Define(self._stage, "/World/Robot/LidarObj")
            #self.lidarObj.CreateRadiusAttr(0.05)
            #self.lidarObj.CreateHeightAttr(0.1)
            #self.lidarObj.AddTranslateOp().Set((0, 0, 1.05))

            '''     
            # PhysX based LiDAR sensor
            self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
            result, lidarPrim = omni.kit.commands.execute(
                "RangeSensorCreateLidar",
                path="/Lidar",
                parent="/World/Robot/LidarObj",
                min_range=0.4,
                max_range=150.0,
                draw_points=(not self._isStereo),
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
            '''
            '''
            create_camera(self._stage, "/World/Robot/LidarObj", "/Lidar", self._camera_k, self._camera_width, self._camera_height)
            rotate_camera(self._stage, "/World/Robot/LidarObj/Lidar", 0.0)

            rp2 = rep.create.render_product("/World/Robot/LidarObj/Lidar", resolution=(self._camera_width, self._camera_height))

            #Access the data through annotators
            self.pointcloud_anno = AnnotatorRegistry.get_annotator("pointcloud")
            self.pointcloud_anno.attach([rp2])
            '''

            #Create pointcloud output directory
            file_path = os.path.join(os.getcwd(), "Isaac-Sim-Playground", "isaacGenSynData" , "_out_pcd", "")
            print(f"Writing pointcloud data to {file_path}")
            self.dir_name_pcd = os.path.dirname(file_path)
            os.makedirs(self.dir_name_pcd, exist_ok=True)
        
        if self.DEBUG_CAMERA:
            mtl_created_list = []
            #Create Materials
            omni.kit.commands.execute(
                "CreateAndBindMdlMaterialFromLibrary",
                mdl_name="OmniPBR.mdl",
                mtl_name="OmniPBR",
                mtl_created_list=mtl_created_list,
            )
            mtl_primStart = self._stage.GetPrimAtPath(mtl_created_list[-1])
            omni.usd.create_material_input(mtl_primStart, "diffuse_color_constant", Gf.Vec3f(0, 1, 0), Sdf.ValueTypeNames.Color3f)
    
            omni.kit.commands.execute(
                "CreateAndBindMdlMaterialFromLibrary",
                mdl_name="OmniPBR.mdl",
                mtl_name="OmniPBR",
                mtl_created_list=mtl_created_list,
            )
            mtl_primEnd = self._stage.GetPrimAtPath(mtl_created_list[-1])
            omni.usd.create_material_input(mtl_primEnd, "diffuse_color_constant", Gf.Vec3f(1, 0, 0), Sdf.ValueTypeNames.Color3f)
            omni.kit.commands.execute(
                "CreateAndBindMdlMaterialFromLibrary",
                mdl_name="OmniPBR.mdl",
                mtl_name="OmniPBR",
                mtl_created_list=mtl_created_list,
            )
            mtl_primPath = self._stage.GetPrimAtPath(mtl_created_list[-1])
            omni.usd.create_material_input(mtl_primPath, "diffuse_color_constant", Gf.Vec3f(1, 0.58, 0), Sdf.ValueTypeNames.Color3f)

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
        if self.DEBUG_CAMERA:
            self._stage.RemovePrim("/World/Path")
        #self._world.play()
        print("> Setup post reset done")

    def get_pointcloud2(self):
        self._world.pause()
        pointcloud = self.lidarInterface.get_point_cloud_data("/World/Robot/LidarObj/Lidar")
        #semantics = self.lidarInterface.get_semantic_data("/World/ZED2/Lidar")
        #print(pointcloud.shape)
        pointcloud = pointcloud.reshape(47700, 3)

        x, y, z = pointcloud[:, 0], pointcloud[:, 1], pointcloud[:, 2]
        mask = (x > -10) & (x < 10) & (y > -10) & (y < 10) & (z > -1)
        pointcloud = pointcloud[mask]

        self._pointclouds.append(pointcloud)

        #Get transform 
        xformable = UsdGeom.Xformable(self._stage.GetPrimAtPath("/World/Robot"))
        transform = xformable.GetLocalTransformation()
        mat = np.array(transform)
        T = np.empty((4, 4))
        T[:3, :3] = mat[:3, :3] # Rotation matrix R
        T[:3, 3] = mat[3, :3] # translation vector t
        T[3, :] = [0, 0, 0, 1] # Homogeneous part
        self._trans_mats.append(T)

        #np.append(pointcloud, T[:3, 3]) # Append point from which the scan was made as reference

        # Create thread to save synthetic LiDAR pointcloud data
        MyThread = Thread(target=save_pcd, args=(pointcloud, f"{self.dir_name_pcd}/synthetic_{self._world.current_time_step_index}_pcd.ply"))
        MyThread.start()

        self._world.play()

    def get_poincloud(self):
        point_cloud = np.array(og.Controller().node("/Render/PostProcess/SDGPipeline/RenderProduct_Isaac_RtxSensorCpuIsaacComputeRTXLidarPointCloud").get_attribute("outputs:pointCloudData").get())
        
        x, y, z = point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2]
        mask = (x > -10) & (x < 10) & (y > -10) & (y < 10) & (z > -1)
        point_cloud = point_cloud[mask]

        #print(point_cloud)

        # Create thread to save synthetic LiDAR pointcloud data
        #MyThread = Thread(target=save_pcd, args=(point_cloud, f"{self.dir_name_pcd}/synthetic_{self._world.current_time_step_index}_pcd.ply"))
        #MyThread.start()

        #if (len(point_cloud) > 1):
        #    print(point_cloud.shape)

    def get_image(self):
        #rep.orchestrator.step() self.rgb_annotators
        # Create thread for each camera annotator to save corresponding synthetic rgb image
        for j, rgb_annot in enumerate(self.rgb_annotators):
            if j == 2:
                file_name = f"{self.dir_name_img}/depth_step_{self._world.current_time_step_index}"
            else:
                file_name = f"{self.dir_name_img}/rp{j}_step_{self._world.current_time_step_index}"
            MyThread = Thread(target=save_rgb, args=(rgb_annot.get_data(), file_name))
            MyThread.start()

    def physics_step(self, step_size):
        if self._i % 100 == 0:
            print("Time step: ", self._i)
        # Early stopping
        if self._i == self._earlyStopping:
            print("Stopping early")
            self._world.stop()

        elif self._i < len(self._points):
            if self._isStereo or self._isLidar:
                set_pose(self._stage, "/World/robot/Robot", self._points[self._i], self._rotations[self._i])

            if self._isLidar:
                #self.get_poincloud()
                pass

            if self._isStereo:
                #self.get_image()
                pass
                
            if self.DEBUG_CAMERA:
                cubeGeom = UsdGeom.Cube.Define(self._stage, "/World/Path/Pos" + str(self._i))
                set_pose(self._stage, "/World/Path/Pos" + str(self._i), self._points[self._i], [0, 0, 0, 1])
                if self._i == 0:
                    cubeGeom.CreateSizeAttr(0.25)
                    cube_mat_shade = UsdShade.Material(self.mtl_primStart)
                elif self._i == len(self._points) - 1:
                    cubeGeom.CreateSizeAttr(0.25)
                    cube_mat_shade = UsdShade.Material(self.mtl_primEnd)
                else:
                    cubeGeom.CreateSizeAttr(0.1)
                    cube_mat_shade = UsdShade.Material(self.mtl_primPath)
                UsdShade.MaterialBindingAPI(cubeGeom).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)
        else:
            #self._world.pause()
            self._world.stop()
        self._i += 1
        #self._world.pause()

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Virtual lab run")
    parser.add_argument(
        "-p", "--path", required=True, type=str, default=None, help=".json file that specifies the pathmap"
    )
    parser.add_argument(
        "-i", "--interpolate", type=str, default=None, help="If provided a .json file of recorded image or pointcloud data with timestamps then interpolate missing poses"
    )
    parser.add_argument(
        "-L", "--lerp", action="store_const", default=False, const=True, help="If specified, a linear interpolation between two poses will be done"
    )
    parser.add_argument(
        "-E", "--early_stopping", type=int, default=-1, help="If specified a value, simulation will be stopped early according to the provided value"
    )
    parser.add_argument(
        "-l", "--lidar", action="store_const", default=False, const=True, help="If specified, a lidar scan will be done"
    )
    parser.add_argument(
        "-s", "--stereo", action="store_const", default=False, const=True, help="If specified, a scan with stereo vision will be done"
    )
    parser.add_argument(
        "--livestream", action="store_const", default=False, const=True, help="If specified, the livestream capability will be activated. Connect via the Streaming Client"
    )
    args, unknown_args = parser.parse_known_args()
    print(args)
    if args.path is None:
        raise ValueError(f"No path specified via --path argument")

    if args.livestream:
        print('> Activate livestream')
        simulation_app.set_setting("/app/window/drawMouse", True)
        simulation_app.set_setting("/app/livestream/proto", "ws")
        simulation_app.set_setting("/app/livestream/websocket/framerate_limit", 120)
        simulation_app.set_setting("/ngx/enabled", False)
        enable_extension("omni.kit.livestream.native")

    enable_extension("omni.isaac.ros2_bridge")
    # Print all available Writers
    #writer_dict = rep.WriterRegistry.get_writers()
    #print(writer_dict.keys())
    
    lab_run = LabRun(args.path, args.interpolate, isLidar=args.lidar, isStereo=args.stereo, isLerp=args.lerp, earlyStopping=args.early_stopping)
    lab_run.load_world()

    if args.lidar:
        enable_extension("omni.isaac.debug_draw")

        _, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/sensor",
            parent="/World/robot/Robot/LidarObj",
            config="RS-Helios-32-5515",
            translation=(0.14, 0, 1.05),
            orientation=Gf.Quatd(0.5, 0.5, -0.5, -0.5),  # Gf.Quatd is w,i,j,k
        )
        _, render_product_path = create_hydra_texture([1, 1], sensor.GetPath().pathString)

        # Create Point cloud publisher pipeline in the post process graph
        writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
        writer.initialize(topicName="point_cloud", frameId="sim_lidar")
        writer.attach([render_product_path])

        if lab_run.DEBUG_LIDAR:
            # Create the debug draw pipeline in the post process graph
            writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
            writer.attach([render_product_path])
        print("> RTX Lidar created")
    
    #lab_run._world.play()

    while simulation_app.is_running():
        #Take a step in the simulation
        lab_run._world.step(render=True)
        if lab_run._world.is_stopped():
            break
        elif lab_run._world.is_playing():
            #Reset the world
            if lab_run._world.current_time_step_index == 0:
                lab_run.reset()

    print('> Closing ...')
    # cleanup
    simulation_app.close()
    #main()