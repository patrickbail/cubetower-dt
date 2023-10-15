#!/bin/bash

# Install pyvista for lab_utility.py
#./python.sh -m pip install open3d
#./python.sh -m pip install opencv-python
./python.sh -m pip install pyvista

# Fix the livestream
sed -i '550s/.*/"omni.isaac.quadruped" = {order = 10}/' ./apps/omni.isaac.sim.python.kit

# Include sensor material map for RTX LiDAR
map='rtx.materialDb.rtSensorNameToIdMap="DefaultMaterial:0;AsphaltStandardMaterial:1;AsphaltWeatheredMaterial:2;VegetationGrassMaterial:3;WaterStandardMaterial:4;GlassStandardMaterial:5;FiberGlassStandardMaterial:6;MetalAlloyMaterial:7;MetalAluminumMaterial:8;MetalAluminumOxidizedMaterial:9;PlasticStandardMaterial:10;RetroMarkingsMaterial:11;RetroSignMaterial:12;RubberStandardMaterial:13;SoilClayMaterial:14;ConcreteRoughMaterial:15;ConcreteSmoothMaterial:16;OakTreeBarkMaterial:17;FabricStandardMaterial:18;PlexiGlassStandardMaterial:19;MetalSilverMaterial:20"'
sed -i '61i\'"$map"'' ./exts/omni.isaac.sensor/config/extension.toml

# Add RTX Sensor Materials properties
echo "styrofoam,FabricStandardMaterial" >> ./kit/rendering-data/RtxSensorMaterialMap.csv
echo "mirror,MetalsilverMaterial" >> ./kit/rendering-data/RtxSensorMaterialMap.csv
echo "clear_glass,GlassStandardMaterial" >> ./kit/rendering-data/RtxSensorMaterialMap.csv

# Add custom helios config to paths
directory_path='"${app}/../cubetower-dt/config/",'
sed -i "58i${directory_path}" ./exts/omni.isaac.sensor/config/extension.toml