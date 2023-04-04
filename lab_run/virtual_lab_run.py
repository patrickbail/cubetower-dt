from omni.isaac.kit import SimulationApp
config = {
     'window_width': "1280",
     'window_height': "720",
     'headless': False,
}
simulation_app = SimulationApp(config)

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import create_new_stage, update_stage
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.range_sensor import _range_sensor 

import json
import argparse
import pptk
import numpy as np
import omni.kit
import omni.usd
from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf, Semantics, UsdPhysics

class LabRun():
    def __init__(self, file_path, file_sim_data, interpolate = False, isLidar = False, isDepthMap = False) -> None:
        self.DEBUG = False
        self._world = None
        self._stage = None
        self._current_tasks = None
        self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
        self._isLidar = isLidar
        self._isDepthMap = isDepthMap
        self._points = []
        self._rotations = []
        self._i = 0
        self._camera_height = 512
        self._camera_width = 896
        self._camera_k = np.array([[364.2601318359375, 0.0, 443.48822021484375],
                                   [0.0, 370.04205322265625, 252.1682891845703],
                                   [0.0, 0.0, 1.0]])
        self._pointclouds = []
        self.interpolate_points(file_path, file_sim_data, interpolate)

    def interpolate_points(self, file_path, file_sim_data, interpolate):
        with open(file_path, 'r', encoding='utf-8') as jsonf:
            path_data = json.load(jsonf)

        with open(file_sim_data, 'r', encoding='utf-8') as jsonf:
            sim_data = json.load(jsonf)

        if interpolate:
            print("> Interpolation starting...")
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
                        self._points.append(path_points[i])
                        self._rotations.append(path_rotation[i])
                        continue
                    if start_t < image_t < end_t:
                        p0 = point_intervals[i][0]
                        p1 = point_intervals[i][1]
                        d = (image_t - start_t)/(end_t - start_t)
                        pd = (1-d)*p0 + d*p1
                        
                        self._points.append(pd)
                        self._rotations.append(path_rotation[i])

            self._points.append(path_points[-1])
            self._rotations.append(path_rotation[-1])
            print("> Interpolation ended")
        else:
            self._points = np.array([pose["pose"]["position"] for pose in path_data])
            #self._points[:, [1,2]] = self._points[:, [2,1]] #Swapping columns
            self._rotations = np.array([pose["pose"]["orientation"] for pose in path_data])
    
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

    def rotate_camera(self, camera_path):
        # Get the camera prim
        camera_prim = self._stage.GetPrimAtPath(camera_path)
        if not camera_prim.IsValid():
            print(f"Camera prim not found at path: {camera_path}")
            return
        
        xformable = UsdGeom.Xformable(camera_prim)
        xformable.ClearXformOpOrder()
        transform = xformable.AddTransformOp()

        # XZY -> XYZ
        matrix = Gf.Matrix3d(
            1, 0, 0,
            0, 0, 1,
            0, -1, 0
        )
        
        #mat = Gf.Matrix4d(
        #    1, 0, 0, 0,
        #    0, 0, -1, 0,
        #    0, -1, 0, 0,
        #    0, 0, 0, 1
        #)
        mat = Gf.Matrix4d()
        mat.SetRotateOnly(matrix.ExtractRotation())
        transform.Set(mat)

    def setup_scene(self):
        print("> Setting up scene...")
        self._stage = omni.usd.get_context().get_stage()
        ##UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.y)

        #PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Y", 100, Gf.Vec3f(0, 0, 0), Gf.Vec3f(1.0))
        self._world.scene.add_default_ground_plane()
        prim_groundplane = self._stage.GetPrimAtPath("/World/defaultGroundPlane")
        self.groundplane = UsdGeom.Xformable(prim_groundplane)
        ##self.groundplane.AddRotateXYZOp().Set((-90, 0, 0))
        #assets_root_path = get_assets_root_path()
        #asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        #add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka_1")

        #self.cubeGeom = UsdGeom.Cube.Define(self._stage, "/World/Cube")
        #self.cubeGeom.CreateSizeAttr(1)
        #self.cubeGeom.AddTranslateOp().Set((0, 1, 1))
        #self.cubeGeom.AddRotateXYZOp().Set((0, 45, -45))
        #cube_mat_shade = UsdShade.Material(mtl_prim)
        #UsdShade.MaterialBindingAPI(self.cubeGeom).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)

        #acopos_asset_path = "omniverse://localhost/Projects/acoposobj_scaled/acoposobj_scaled.usd"
        acopos_asset_path = "./lab_run/acoposobj_scaled.usd"
        self.acoposObj = add_reference_to_stage(usd_path=acopos_asset_path, prim_path="/World/Acopos")
        ##self.set_pose("/World/Acopos", [3, 1, -1], [0, 0, -0.70711, 0.70711])
        self.set_pose("/World/Acopos", [3, 1, 1], [0.5, -0.5, -0.5, 0.5])
        collisionAPI = UsdPhysics.CollisionAPI.Apply(self.acoposObj)
        sem = Semantics.SemanticsAPI.Apply(self.acoposObj, "Semantics")
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set("class")
        sem.GetSemanticDataAttr().Set("Acopos")

        #zed2_asset_path = "omniverse://localhost/Projects/ZED2_scaled/ZED2_scaled.usd"
        zed2_asset_path = "./lab_run/ZED2_scaled.usd"
        self.zed2Obj = add_reference_to_stage(usd_path=zed2_asset_path, prim_path="/World/ZED2")
        ##self.set_pose("/World/ZED2", [0, 1, 0], [0, 0, 0, 1])
        self.set_pose("/World/ZED2", [0, 0, 1], [0, 0, 0, 1])
        #self.cameraObj = UsdGeom.Cube.Define(stage, "/World/CamerObj")
        #self.cameraObj.CreateSizeAttr(0.25)
        #self.cameraObj.AddTranslateOp().Set((0, 1, 0))
        #self.cameraObj.AddOrientOp().Set(Gf.Quatf(1, [0, 0, 0]))

        if self._isDepthMap:
            print("> Setting up Stereo Camera...")

            self.left_camera = UsdGeom.Camera.Define(self._stage, "/World/ZED2/CameraLeft")
            #self.left_camera = UsdGeom.Camera.Define(stage, "/World/CamerObj/CameraLeft")
            self.left_camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
            self.left_camera.CreateFocalLengthAttr().Set(self._camera_k[0,0])
            self.left_camera.CreateHorizontalApertureAttr().Set(self._camera_width)
            self.left_camera.CreateVerticalApertureAttr().Set(self._camera_height)

            self.left_camera.CreateClippingRangeAttr().Set((0.1,100000))

            self.rotate_camera("/World/ZED2/CameraLeft")


        if self._isLidar:
            print("> Setting up Lidar Sensor...")
            self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
            result, lidarPrim = omni.kit.commands.execute(
                "RangeSensorCreateLidar",
                path="/Lidar",
                parent="/World/ZED2",
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

        if self.DEBUG:
            #Create Materials
            mtl_created_list = []
            omni.kit.commands.execute(
                "CreateAndBindMdlMaterialFromLibrary",
                mdl_name="OmniPBR.mdl",
                mtl_name="OmniPBR",
                mtl_created_list=mtl_created_list,
            )
            self.mtl_primStart = self._stage.GetPrimAtPath(mtl_created_list[0])
            omni.usd.create_material_input(self.mtl_primStart, "diffuse_color_constant", Gf.Vec3f(0, 1, 0), Sdf.ValueTypeNames.Color3f)
    
            omni.kit.commands.execute(
                "CreateAndBindMdlMaterialFromLibrary",
                mdl_name="OmniPBR.mdl",
                mtl_name="OmniPBR",
                mtl_created_list=mtl_created_list,
            )
            self.mtl_primEnd = self._stage.GetPrimAtPath(mtl_created_list[1])
            omni.usd.create_material_input(self.mtl_primEnd, "diffuse_color_constant", Gf.Vec3f(1, 0, 0), Sdf.ValueTypeNames.Color3f)
            omni.kit.commands.execute(
                "CreateAndBindMdlMaterialFromLibrary",
                mdl_name="OmniPBR.mdl",
                mtl_name="OmniPBR",
                mtl_created_list=mtl_created_list,
            )
            self.mtl_primPath = self._stage.GetPrimAtPath(mtl_created_list[2])
            omni.usd.create_material_input(self.mtl_primPath, "diffuse_color_constant", Gf.Vec3f(1, 0.58, 0), Sdf.ValueTypeNames.Color3f)
            

        print("> Setup done")
    
    def setup_post_load(self):
        print("> Setup post load...")
        #self.camera_obj.disable_rigid_body_physics()
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        #self._world.play()
        print("> Setup post load done")

    def setup_post_reset(self):
        print("> Setup post reset...")
        self._i = 0
        ##self.set_pose("/World/ZED2", [0, 1, 0], [0, 0, 0, 1])
        self.set_pose("/World/ZED2", [0, 0, 1], [0, 0, 0, 1])
        ##self.groundplane.GetOrderedXformOps()[-1].Set((-90, 0, 0))
        if self._isLidar:
            self._pointclouds = []
        if self.DEBUG:
            self._stage.RemovePrim("/World/Path")
        #self._world.play()
        print("> Setup post reset done")
    
    def set_pose(self, prim_path, position=None, orientation=None):
        prim = self._stage.GetPrimAtPath(str(prim_path))
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        transform = xform.AddTransformOp()
        mat = Gf.Matrix4d()
        if position is not None:
            mat.SetTranslateOnly(Gf.Vec3d(*position))
        if orientation is not None:
            mat.SetRotateOnly(Gf.Rotation(Gf.Quaternion(orientation[-1], Gf.Vec3d(*orientation[:-1]))))
            #mat.SetRotateOnly(Gf.Rotation(Gf.Quaternion(orientation[-1], Gf.Vec3d(orientation[0], orientation[2], orientation[1]))))
        transform.Set(mat)

    def get_pointcloud(self):
        self._world.pause()
        pointcloud = self.lidarInterface.get_point_cloud_data("/World/ZED2/Lidar")
        #semantics = self.lidarInterface.get_semantic_data("/World/ZED2/Lidar")
        pointcloud = pointcloud.reshape(47700, 3)
        self._pointclouds.append(pointcloud)
        #print("Point Cloud", pointcloud)
        #print("Semantic ID", semantics)
        #print(pointcloud.shape)
        self._world.play()

    def physics_step(self, step_size):
        #self.cubeGeom.GetOrderedXformOps()[-1].Set((20*self._world.current_time, 45, 20*self._world.current_time))
        #self.cubeGeom.SetRotate((0, 0, self._world.current_time))
        if self._i < len(self._points):
            ##self._points[self._i][1] = 1
            self._points[self._i][2] = 1
            if self._isLidar:
                self.set_pose("/World/ZED2", self._points[self._i], [0, 0, 0, 1])
            else:
                self.set_pose("/World/ZED2", self._points[self._i], self._rotations[self._i])
            #self.set_pose("/World/CamerObj", self._points[self._i], self._rotations[self._i])

            if self.DEBUG:
                cubeGeom = UsdGeom.Cube.Define(self._stage, "/World/Path/Pos" + str(self._i))
                self.set_pose("/World/Path/Pos" + str(self._i), self._points[self._i], [0, 0, 0, 1])
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
            
            
            #self.cameraObj.GetOrderedXformOps()[0].Set(Gf.Vec3d(self._points[self._i][0], self._points[self._i][1], 1))
            #self.cameraObj.GetOrderedXformOps()[-1].Set(Gf.Vec3d(self.euler_from_quaternion(self._rotations[self._i])))
            #print(self._rotations[self._i][-1])
            #print(self._rotations[self._i][:-1])
            #self.cameraObj.GetOrderedXformOps()[-1].Set(Gf.Quatf(self._rotations[self._i][-1], 
            #                                                     self._rotations[self._i][0], 
            #                                                     self._rotations[self._i][1], 
            #                                                     self._rotations[self._i][2]))
            if self._isLidar:
                self.get_pointcloud()
        self._i += 1
        #self._world.pause()

def main():
    parser = argparse.ArgumentParser("Virtual lab run")
    parser.add_argument(
        "-p", "--path", required=True, type=str, default=None, help=".json file that specifies the pathmap"
    )
    parser.add_argument(
        "-s", "--sim_data", required=True, type=str, default=None, help=".json file of recorded image or pointcloud data with timestamps"
    )
    parser.add_argument(
        "-i", "--interpolate", action="store_const", default=False, const=True, help="If specified, missing poses will be interpolated"
    )
    parser.add_argument(
        "-l", "--lidar", action="store_const", default=False, const=True, help="If specified, a lidar scan will be done"
    )
    parser.add_argument(
        "-d", "--depth_map", action="store_const", default=False, const=True, help="If specified, a depth map will be generated"
    )
    args, unknown_args = parser.parse_known_args()
    print(args)
    if args.path is None:
        raise ValueError(f"No path specified via --path argument")
    
    if args.sim_data is None:
        raise ValueError(f"No simulation data specified via --sim_data argument")
    
    lab_run = LabRun(args.path, args.sim_data, args.interpolate, isLidar=args.lidar, isDepthMap=args.depth_map)
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
    
    #pointcloud = np.concatenate(lab_run._pointclouds)
    #pointcloud = lab_run._pointclouds[1]
    #v = pptk.viewer(pointcloud)

if __name__ == "__main__":
    main()