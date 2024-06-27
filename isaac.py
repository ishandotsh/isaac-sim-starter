from omni.isaac.kit import SimulationApp

app_config = {
     "width": "1280",
     "height": "720",
     "headless": False,
     "renderer": "RayTracedLighting",
}

app = SimulationApp(app_config)

from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

from omni.isaac.core import World
world = World(stage_units_in_meters=1.0)

assets_root_path = get_assets_root_path()

def spawn_environment(world_type: str = "ground_plane"):
     if world_type == "ground_plane":
          world.scene.add_default_ground_plane()
     elif world_type == "warehouse":
          warehouse_asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
          warehouse_prim = add_reference_to_stage(usd_path=warehouse_asset_path, prim_path="/World/Warehouse")
     elif world_type == "warehouse_shelves":
          warehouse_asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
          warehouse_prim = add_reference_to_stage(usd_path=warehouse_asset_path, prim_path="/World/Warehouse")
     elif world_type == "warehouse_full":
          warehouse_asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
          warehouse_prim = add_reference_to_stage(usd_path=warehouse_asset_path, prim_path="/World/Warehouse")

spawn_environment("ground_plane")

# https://openusd.org/release/api/index.html
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics, Usd, UsdGeom
import omni.kit.commands

stage = omni.usd.get_context().get_stage()
# difference between scene, stage, and world
# https://forums.developer.nvidia.com/t/scene-vs-stage-vs-world/270115

# Physics 
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
physxSceneAPI.CreateEnableCCDAttr(True)
physxSceneAPI.CreateEnableStabilizationAttr(True)
physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
physxSceneAPI.CreateSolverTypeAttr("TGS") # or PGS

# Lighting
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(500)

# URDF
from omni.importer.urdf import _urdf

status, robot_import_config = omni.kit.commands.execute("URDFCreateImportConfig")
robot_import_config.merge_fixed_joints = False
robot_import_config.convex_decomp = True # Collision
robot_import_config.import_inertia_tensor = True
robot_import_config.fix_base = False
robot_import_config.distance_scale = 1

robot_import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_NONE

robot_urdf_path = "current_path/assets/turtlebot3_waffle_pi/urdf/turtlebot3_waffle_pi.urdf"
status, robot_prim_path = omni.kit.commands.execute(
     "URDFParseAndImportFile",
     urdf_path=robot_urdf_path,
     import_config=robot_import_config,
     get_articulation_root=True,
)

# Set joint properties

from omni.isaac.core.utils.stage import is_stage_loading
while is_stage_loading():
    app.update()

from typing import List

velocity_driven_joints = ['wheel_left_joint', 'wheel_right_joint']
position_driven_joints = []
undriven_joints = []

def get_robot_description(robot_prim: Usd.Prim) -> List[Usd.Prim]:
     robot_description = []

     def gather_paths(prim: Usd.Prim):
          for child_prim in prim.GetAllChildren():
               if child_prim.GetTypeName() not in ["Mesh", "Shader", "Material", "Scope"]:
                    # print(child_prim.GetPath().pathString, " -> ", child_prim.GetTypeName())
                    robot_description.append(child_prim)
                    gather_paths(child_prim)

     gather_paths(robot_prim)
     return robot_description

robot_prim = stage.GetPrimAtPath(robot_prim_path).GetParent()
robot_tree = get_robot_description(robot_prim)

# Set velocity, position, or undriven joints
for prim in robot_tree:
     prim_type = prim.GetTypeName()
     prim_path = prim.GetPath()
     if prim_type == "PhysicsRevoluteJoint":
          # print("At ", prim.GetPath().pathString, end=" ")
          prim_name = prim_path.pathString.split("/")[-1].strip()
          if prim_name in velocity_driven_joints:
               # print("-> Setting Velocity")
               drive = UsdPhysics.DriveAPI.Get(prim, "angular")
               drive.GetStiffnessAttr().Set(0)
               drive.GetDampingAttr().Set(1e10)
          elif prim_name in position_driven_joints:
               # print("-> Setting Position")
               drive = UsdPhysics.DriveAPI.Get(prim, "angular")
               drive.GetStiffnessAttr().Set(1e10)
               drive.GetDampingAttr().Set(0)
          else:
               # print("-> Setting Undriven")
               # print("Prop Names: ")
               # print(prim.GetPropertyNames())
               prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
               prim.GetProperty("physxJoint:jointFriction").Set(0.001)
               prim.GetProperty("physxJoint:maxJointVelocity").Set(4.0)

# Lidar
lidar_path = 'lidar'
lidar_parent = str(robot_prim.GetPrimPath()) + '/base_scan'
lidar_config = "Example_Rotary" # $ISAAC_PATH/exts/omni.isaac.sensor/data/lidar_configs for a list of other configs

_, sensor = omni.kit.commands.execute(
     "IsaacSensorCreateRtxLidar",
     path=lidar_path,
     parent=lidar_parent,
     config=lidar_config,
     orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0) # W, X, Y, Z
)

# IMU
imu_path = 'imu'
imu_parent = str(robot_prim.GetPrimPath()) + '/imu_link'

for prim in stage.TraverseAll():
    if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
        current_physics_prim = prim
physx_scene_api = PhysxSchema.PhysxSceneAPI(current_physics_prim)
current_physics_frequency = physx_scene_api.GetTimeStepsPerSecondAttr().Get()
dt = 1.0 / current_physics_frequency # default = 1.0 / 60

imu_result, imu_prim = omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path=imu_path,
    parent=imu_parent,
    sensor_period=dt,
    visualize=True,
    linear_acceleration_filter_size=1,
    angular_velocity_filter_size=1,
    orientation_filter_size=1,
)


# Camera
from omni.isaac.sensor import Camera

camera_path = str(robot_prim.GetPrimPath()) + '/camera_rgb_frame/camera'

camera_prim = Camera(
     prim_path=camera_path,
     frequency=20,
     resolution=(256, 256),
)

# Actiongraph
from omni.isaac.core.utils.extensions import enable_extension # For ROS
enable_extension("omni.isaac.ros_bridge")

import omni.graph.core as og
import usdrt.Sdf
controller = og.Controller
keys = controller.Keys

# https://docs.omniverse.nvidia.com/kit/docs/omni.graph/latest/omni.graph.core/omni.graph.core.Controller.html#omni.graph.core.Controller.edit

graph_config = {"graph_path": "/ActionGraph", "evaluator_name": "execution"}
graph, _, _, _ = controller.edit(
     graph_config,
     {
          keys.CREATE_NODES: [
               ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
               ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
               ("PublishClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
               
          ],
          keys.CONNECT: [
               ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
               ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),

          ],
          keys.SET_VALUES: [
               ("PublishClock.inputs:topicName", "/clock"),

          ]
     }
)

from utils import *

add_camera(graph, camera_path=camera_path, 
          camera_name="my_camera", colorFrameId="camera", colorTopicName="camera/color",
          infoFrameId="camera", infoTopicName="camera/camera_info")

add_lidar(graph, lidar_parent+"/"+lidar_path, "my_lidar", 
          "laser", "scan", "laser_scan")

add_imu(graph, imu_parent+'/'+imu_path, "IMU", "imu")

add_diff_teleop(graph, str(robot_prim.GetPrimPath())+'/base_footprint', velocity_driven_joints, 0.26, 0.3, 0.066, "cmd_vel")

add_odometry(graph, "ComputeOdometry", str(robot_prim.GetPrimPath())+'/base_footprint',
     "base_footprint", "odom", "odom")

# manual tf tree
# base_footprint -> base_link
add_tf(graph, "PubTFBaseStatic", [str(robot_prim.GetPrimPath())+'/base_link'], "tf_static", str(robot_prim.GetPrimPath())+'/base_footprint')

# defines base_link -> (all other joints)
static_links = [
     "camera_link", "caster_back_left_link",
     "caster_back_right_link", "imu_link", "base_scan",
]
path_prefix = str(robot_prim.GetPrimPath()) + '/'
static_links_paths = [path_prefix + link for link in static_links]

add_tf(graph, "PubTFStatic", static_links_paths, "tf_static", str(robot_prim.GetPrimPath())+'/base_link')

# base_link -> wheel links on /tf
wheel_links = ["wheel_left_link", "wheel_right_link"]
wheel_links_paths = [path_prefix + link for link in wheel_links]

add_tf(graph, "PubTFWheels", wheel_links_paths, "tf", str(robot_prim.GetPrimPath())+'/base_link')

from omni.isaac.core import SimulationContext
simulation_context = SimulationContext()
simulation_context.play()

while app.is_running():
    try:
        simulation_context.step(render=True)

    except KeyboardInterrupt:
        break

simulation_context.stop()
app.close()
