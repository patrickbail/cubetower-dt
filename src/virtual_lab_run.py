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
    "renderer": "RayTracedLighting",  # Can also be PathTracing, RayTracedLighting
    "anti_aliasing": 3, # 3 DLSS
    # All following settings are for PathTracing only
    "samples_per_pixel_per_frame": 64,
    "denoiser": True,
    "max_bounces": 4,
    "max_specular_transmission_bounces": 6,
    "max_volume_bounces": 4,
    "open_usd": None,
    "livesync_usd": None,
}
# Load SimulationApp first, before importing isaac-core or kit libraries
simulation_app = SimulationApp(config)

#python
import os
import json
import argparse
import numpy as np
from threading import Thread
from lab_utility import create_camera, set_pose, load_data, save_rgb, save_pcd

#isaac-core
from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage, update_stage, add_reference_to_stage, open_stage
from omni.isaac.core.utils.extensions import enable_extension 
from omni.isaac.core.utils.render_product import create_hydra_texture
from omni.isaac.range_sensor import _range_sensor

#omniverse
import carb
import omni.kit
import omni.usd
import omni.graph.core as og
import omni.replicator.core as rep
from omni.replicator.core import AnnotatorRegistry
from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf, Semantics, UsdPhysics

class LabRun():
    def __init__(self, config, file_path, file_sensor = None, isLidar = False, isPhysXLidar = False, isStereo = False, 
                 isLerp = False, earlyStopping = -1, isInterpolate = True, saveToFile = True) -> None:
        self.DEBUG_CAMERA = False
        self.DEBUG_LIDAR = False
        self._world = None
        self._stage = None
        self._current_tasks = None
        self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
        self._isLidar = isLidar
        self._isPhysXLidar = isPhysXLidar
        self._isStereo = isStereo
        self._isLerp = isLerp
        self._earlyStopping = earlyStopping
        self._saveToFile = saveToFile
        self._isInterpolate = isInterpolate
        self._i = 0 # Time step index
        self._camera_height = config["Sensors"]["Camera"]["height"]
        self._camera_width = config["Sensors"]["Camera"]["width"]
        self._camera_k = np.array(config["Sensors"]["Camera"]["Intrinsic"])
        self._config = config
        self.hydra_texture = None

        self._points, self._rotations = load_data(file_path, file_sensor, isLerp, isInterpolate)
    
    def load_world(self):
        print("> Load simulation...")
        # Load USD stage here
        #file_path = "./Isaac-Sim-Playground/Assets/USD-Files/final_cube_setup.usd"
        file_path = self._config["Scene"]["Stage USD"]
        open_stage(file_path)
        if World.instance() is None:
            #create_new_stage()
            self._world = World(**self._world_settings)
            #await self._world.initialize_simulation_context_async()
            self.setup_scene()
        else:
            self._world = World.instance()
            self.setup_scene()
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
        # Get stage of current scene
        self._stage = omni.usd.get_context().get_stage()

        # Add USD refrence of robot here
        #file_path = "./Isaac-Sim-Playground/Assets/USD-Files/robot.usd"
        file_path = self._config["Scene"]["Robot USD"]
        add_reference_to_stage(
            usd_path=file_path, prim_path="/World"
        )

        if self._isStereo:
            print("> Setting up Stereo Camera...")

            # Enable opt in feature for augmenting render pipeline
            settings_i = carb.settings.acquire_settings_interface()
            settings_i.set_bool("/app/omni.graph.scriptnode/opt_in", True)

            # Add refrence of ZED 2 camera model 
            #zed2_asset_path = "./Isaac-Sim-Playground/Assets/USD-Files/ZED2_scaled.usd" # Local version
            zed2_asset_path = self._config["Sensors"]["Camera"]["USD"]
            add_reference_to_stage(usd_path=zed2_asset_path, prim_path="/World/Robot/ZED2")
            # Rotate ZED 2 camera, make it look down the +X axis with +Z upwards since cameras look down the -Z axis with +Y upwards
            zed_prim = self._stage.GetPrimAtPath("/World/Robot/ZED2")            
            xformable = UsdGeom.Xformable(zed_prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp().Set(Gf.Vec3d([0.128, 0, 0.7])) # Formally 0.4 x
            xformable.AddRotateXZYOp().Set(Gf.Vec3d(0, -90, -89.2))

            # Create left and right stereo cameras
            create_camera(self._stage, "/World/Robot/ZED2", "/CameraLeft", self._camera_k, 
                          self._camera_width, self._camera_height, [0.06, 0, 0], "left")
            create_camera(self._stage, "/World/Robot/ZED2", "/CameraRight", self._camera_k, 
                          self._camera_width, self._camera_height, [-0.06, 0, 0], "right")
            create_camera(self._stage, "/World/Robot/ZED2", "/DepthCamera", self._camera_k, 
                          self._camera_width, self._camera_height, [0., 0, 0])
            
            # Create gaussian noise augmentation
            def image_gaussian_noise_np(data_in: np.ndarray, kernel_seed, mu: float = 0.0, sigma: float = 25.0):
                np.random.seed(kernel_seed)
                gaussian_noise = np.random.normal(mu, sigma, data_in.shape)
                return np.clip(data_in + gaussian_noise, 0, 255).astype(np.uint8)
            
            gn_augm = rep.annotators.Augmentation.from_function(image_gaussian_noise_np, kernel_seed=123, sigma=0.75)
            
            # Create render products
            rp0 = rep.create.render_product("/World/Robot/ZED2/CameraLeft", resolution=(self._camera_width, self._camera_height))
            rp1 = rep.create.render_product("/World/Robot/ZED2/CameraRight", resolution=(self._camera_width, self._camera_height))
            rp2 = rep.create.render_product("/World/Robot/ZED2/DepthCamera", resolution=(self._camera_width, self._camera_height))

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

            # Access the data through annotators
            self.rgb_annotators = []
            for i, rp in enumerate([rp0, rp1, rp2]):
            #for i, rp in enumerate([rp0]):
                if i == 2:
                    annot = AnnotatorRegistry.get_annotator("distance_to_image_plane")
                else:
                    annot = AnnotatorRegistry.get_annotator("rgb")
                    annot.augment(gn_augm)
                annot.attach([rp])
                self.rgb_annotators.append(annot)

            #Create annotator output directory
            if self._isInterpolate:
                file_path = os.path.join(os.getcwd(), "Isaac-Sim-Playground", "_out_no_interpolation_image", "")
            else: 
                file_path = os.path.join(os.getcwd(), "Isaac-Sim-Playground", "_out_image", "")
            print(f"Writing annotator data to {file_path}")
            self.dir_name_img = os.path.dirname(file_path)
            os.makedirs(self.dir_name_img, exist_ok=True)

        if self._isLidar:
            print("> Setting up RTX Lidar Sensor...")

            # Create RTX LiDAR sensor
            _, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path="/RTXLidar",
                parent="/World/Robot/LidarObj",
                config=self._config["Sensors"]["LiDAR"]["config"],
                translation=(0.04, 0, 1.1),
                #orientation=Gf.Quatd(0.5, 0.5, -0.5, -0.5),  # Gf.Quatd is w,i,j,k
                orientation=Gf.Quatd(0.65328, 0.65328, -0.2706, -0.2706), # -90° rotation around Z and Y axis already included
            )
            self.hydra_texture, render_product_path = create_hydra_texture([1, 1], sensor.GetPath().pathString)

            # Create Point cloud publisher pipeline in the post process graph
            writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
            writer.initialize(topicName="point_cloud", frameId="sim_lidar")
            writer.attach([render_product_path])
        
            if self._isInterpolate:
                file_path = os.path.join(os.getcwd(), "Isaac-Sim-Playground", "_out_no_interpolation_pcd", "")
            else: 
                file_path = os.path.join(os.getcwd(), "Isaac-Sim-Playground", "_out_pcd", "")
            print(f"Writing pointcloud data to {file_path}")
            self.dir_name_pcd = os.path.dirname(file_path)
            os.makedirs(self.dir_name_pcd, exist_ok=True)
            print("> RTX Lidar created")

        if self._isPhysXLidar:
            print("> Setting up PhysX Lidar Sensor...")

            # Create PhysX based LiDAR sensor
            self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
            result, lidarPrim = omni.kit.commands.execute(
                "RangeSensorCreateLidar",
                path="/PhysXLidar",
                parent="/World/Robot/LidarObj",
                min_range=0.4,
                max_range=150.0,
                draw_points=self.DEBUG_LIDAR,
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
            lidarPrim.ClearXformOpOrder()
            lidarPrim.AddTranslateOp().Set(Gf.Vec3d(0.04, 0, 1.1))
            lidarPrim.AddRotateXZYOp().Set(Gf.Vec3d(0, 0, 45))

            # Add collision volumes to the cube tower
            cubeprim = self._stage.GetPrimAtPath("/World/CubeTower")
            collisionAPI = UsdPhysics.CollisionAPI.Apply(cubeprim)
            # Semantics only for labelling data
            #sem = Semantics.SemanticsAPI.Apply(cubeprim, "Semantics")
            #sem.CreateSemanticTypeAttr()
            #sem.CreateSemanticDataAttr()
            #sem.GetSemanticTypeAttr().Set("class")
            #sem.GetSemanticDataAttr().Set("cube")

            # Create pointcloud output directory
            if self._isInterpolate:
                file_path = os.path.join(os.getcwd(), "Isaac-Sim-Playground", "_out_no_interpolation_physx_pcd", "")
            else: 
                file_path = os.path.join(os.getcwd(), "Isaac-Sim-Playground", "_out_physx_pcd", "")
            print(f"Writing pointcloud data to {file_path}")
            self.dir_name_physx_pcd = os.path.dirname(file_path)
            os.makedirs(self.dir_name_physx_pcd, exist_ok=True)
            print("> PhysX Lidar created")
        
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

        if lab_run.DEBUG_LIDAR:
            # Create the debug draw pipeline in the post process graph
            enable_extension("omni.isaac.debug_draw")
            writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
            writer.attach([render_product_path])

        print("> Setup done")
    
    def setup_post_load(self):
        print("> Setup post load...")
        # Add physics callback function
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        #self._world.play()
        print("> Setup post load done")

    def setup_post_reset(self):
        print("> Setup post reset...")
        self._i = 0
        set_pose(self._stage, "/World/Robot", self._points[0], self._rotations[0])
        if self.DEBUG_CAMERA:
            self._stage.RemovePrim("/World/Path")
        #self._world.play()
        print("> Setup post reset done")

    def get_pointcloud_physx(self, index_step):
        #self._world.pause()
        pointcloud = self.lidarInterface.get_point_cloud_data("/World/Robot/LidarObj/PhysXLidar")
        #semantics = self.lidarInterface.get_semantic_data("/World/Robot/LidarObj/PhysXLidar")
        pointcloud = pointcloud.reshape(pointcloud.shape[0]*pointcloud.shape[1], 3)

        # Mask out intial unimportant parts of the resulting point cloud data
        x, y, z = pointcloud[:, 0], pointcloud[:, 1], pointcloud[:, 2]
        mask = (x > -10) & (x < 10) & (y > -10) & (y < 10) & (z > -1)
        pointcloud = pointcloud[mask]

        # Create thread to save synthetic PhysX LiDAR pointcloud data
        MyThread = Thread(target=save_pcd, args=(pointcloud, f"{self.dir_name_physx_pcd}/synthetic_{index_step}_physx_pcd.ply"))
        MyThread.start()

        #self._world.play()

    def get_poincloud(self, index_step):
        pointcloud = np.array(og.Controller().node("/Render/PostProcess/SDGPipeline/RenderProduct_Isaac_RtxSensorCpuIsaacComputeRTXLidarPointCloud").get_attribute("outputs:pointCloudData").get())

        # Create thread to save synthetic LiDAR pointcloud data
        MyThread = Thread(target=save_pcd, args=(pointcloud, f"{self.dir_name_pcd}/synthetic_{index_step}_pcd.ply"))
        MyThread.start()

    def get_image(self, index_step):
        # Create thread for each camera annotator to save corresponding synthetic rgb or depth image
        for j, rgb_annot in enumerate(self.rgb_annotators):
            if j == 0:
                file_name = f"{self.dir_name_img}/left_step_{index_step}"
            elif j == 1:
                file_name = f"{self.dir_name_img}/right_step_{index_step}"
            else:
                file_name = f"{self.dir_name_img}/depth_step_{index_step}"
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
            # At each physics step, position and orient robot to collect next batch of data
            if self._isStereo or self._isLidar or self._isPhysXLidar:
                set_pose(self._stage, "/World/Robot", self._points[self._i], self._rotations[self._i])

            if self._isLidar and self._saveToFile:
                self.get_poincloud(self._i)
            
            if self._isPhysXLidar and self._saveToFile:
                self.get_pointcloud_physx(self._i)

            if self._isStereo and self._saveToFile:
                self.get_image(self._i)
                
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
        "-p", "--path", type=str, default=None, help="Provided .json file that specifies the path, overrides trajectory file in config"
    )
    parser.add_argument(
        "-s", "--sim_data", type=str, default=None, help="If provided a .json file of recorded image or pointcloud data with timestamps, then override sensor timestamps file in config"
    )
    parser.add_argument(
        "--no_interpolation", action="store_const", default=True, const=False, help="If specified instead of interpolating poses, assign pose of nearest timestamp match"
    )
    parser.add_argument(
        "--lerp_path", action="store_const", default=False, const=True, help="If specified, a linear interpolation between two poses provided in a .json file will be done"
    )
    parser.add_argument(
        "-e", "--early_stopping", type=int, default=-1, help="If specified a value, simulation will be stopped early according to the provided value"
    )
    parser.add_argument(
        "-l", "--rtx_lidar", action="store_const", default=False, const=True, help="If specified, a lidar scan will be done with the RTX LiDAR"
    )
    parser.add_argument(
        "--physx_lidar", action="store_const", default=False, const=True, help="If specified, a lidar scan will be done with the PhysX LiDAR"
    )
    parser.add_argument(
        "-c", "--stereo_camera", action="store_const", default=False, const=True, help="If specified, a scan with stereo vision will be done"
    )
    parser.add_argument(
        "--livestream", action="store_const", default=False, const=True, help="If specified, the livestream capability will be activated. Connect via the Streaming Client"
    )
    parser.add_argument(
        "--save_to_file", action="store_const", default=False, const=True, help="If specified synthetica data will be saved on the harddrive"
    )
    args, unknown_args = parser.parse_known_args()
    print(args)

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

    # Load config file
    with open("./Isaac-Sim-Playground/config/simulation_config.json", 'r', encoding='utf-8') as jsonf:
        config = json.load(jsonf)
    
    if args.path:
        config["Simulation"]["Path"] = args.path
    if args.sim_data:
        config["Simulation"]["Sensor Timestamps"] = args.sim_data
    
    lab_run = LabRun(config, config["Simulation"]["Path"], file_sensor=config["Simulation"]["Sensor Timestamps"], 
                     isLidar=args.rtx_lidar, isPhysXLidar=args.physx_lidar, isStereo=args.stereo_camera,
                     isLerp=args.lerp_path, earlyStopping=args.early_stopping,
                     isInterpolate=args.no_interpolation, saveToFile=args.save_to_file)
    lab_run.load_world()

    # Render settings 
    #carb_settings = carb.settings.acquire_settings_interface()
    #carb_settings.set("/rtx/hydra/faceCulling/enabled", True)
    #carb_settings.set("/rtx/reflections/maxReflectionBounces", 3)
    #carb_settings.set("/rtx/post/motionblur/enabled", True)
    #carb_settings.set("/rtx/post/lensFlares/enabled", True)
    #carb_settings.set("/rtx/post/lensFlares/flareScale", 0.5)
    
    if not args.livestream:
        lab_run._world.play()

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