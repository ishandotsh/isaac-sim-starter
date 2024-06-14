import omni.graph.core as og
import usdrt.Sdf
from typing import List, Optional

def add_camera(
        graph: og.Graph,
        camera_path,
        camera_name,
        colorFrameId,
        colorTopicName,
        infoFrameId,
        infoTopicName,
        depthFrameId = None,
        depthTopicName = None):

    isDepthCam = False
    if(type(depthFrameId) != type(depthTopicName)):
        print("You must provide both depthFrameId and depthTopicName to create a depth camera")
        print("Skipping creating depth camera")
        isDepthCam = False
    if(type(depthFrameId) == str):
        isDepthCam = True

    graph_path = graph.get_path_to_graph()
    tick_node_path = graph_path + '/OnPlaybackTick'
    create_render_path = graph_path + '/CreateRenderProduct' + camera_name
    rgb_path = graph_path + '/CameraHelperRgb' + camera_name
    info_path = graph_path + '/CameraHelperInfo' + camera_name
    
    controller = og.Controller
    controller.create_node(create_render_path, "omni.isaac.core_nodes.IsaacCreateRenderProduct")
    controller.create_node(rgb_path,    "omni.isaac.ros_bridge.ROS1CameraHelper")
    controller.create_node(info_path,   "omni.isaac.ros_bridge.ROS1CameraHelper")

    controller.connect(tick_node_path+'.outputs:tick',create_render_path+'.inputs:execIn')
    controller.connect(create_render_path+'.outputs:execOut',rgb_path+'.inputs:execIn')
    controller.connect(create_render_path+'.outputs:execOut',info_path+'.inputs:execIn')
    controller.connect(create_render_path+'.outputs:renderProductPath', rgb_path+'.inputs:renderProductPath')
    controller.connect(create_render_path+'.outputs:renderProductPath', info_path+'.inputs:renderProductPath')

    controller.attribute(create_render_path+'.inputs:cameraPrim').set([usdrt.Sdf.Path(camera_path)])
    controller.attribute(rgb_path+'.inputs:frameId').set(colorFrameId)
    controller.attribute(info_path+'.inputs:frameId').set(infoFrameId)
    controller.attribute(rgb_path+'.inputs:topicName').set(colorTopicName)
    controller.attribute(info_path+'.inputs:topicName').set(infoTopicName)
    controller.attribute(rgb_path+'.inputs:type').set("rgb")
    controller.attribute(info_path+'.inputs:type').set("camera_info")

    if(isDepthCam):
        depth_path = graph_path + '/CameraHelperDepth' + camera_name
        controller.create_node(depth_path,   "omni.isaac.ros_bridge.ROS1CameraHelper")
        controller.connect(create_render_path+'.outputs:execOut',depth_path+'.inputs:execIn')
        controller.connect(create_render_path+'.outputs:renderProductPath', depth_path+'.inputs:renderProductPath')
        controller.attribute(depth_path+'.inputs:frameId').set(depthFrameId)
        controller.attribute(depth_path+'.inputs:topicName').set(depthTopicName)
        controller.attribute(depth_path+'.inputs:type').set("depth")


def add_lidar(
        graph: og.Graph,
        lidar_path,
        lidar_name,
        lidarFrameId,
        lidarTopicName,
        lidarType = "laser_scan" # or point_cloud
    ):
    graph_path = graph.get_path_to_graph()
    tick_node_path = graph_path + '/OnPlaybackTick'
    create_render_path = graph_path + '/CreateRenderProduct' + lidar_name
    lidar_helper_path = graph_path + '/LidarHelper' + lidar_name

    controller = og.Controller
    controller.create_node(create_render_path, "omni.isaac.core_nodes.IsaacCreateRenderProduct")
    controller.create_node(lidar_helper_path,    "omni.isaac.ros_bridge.ROS1RtxLidarHelper")

    controller.connect(tick_node_path+'.outputs:tick',create_render_path+'.inputs:execIn')
    controller.connect(create_render_path+'.outputs:execOut',lidar_helper_path+'.inputs:execIn')
    controller.connect(create_render_path+'.outputs:renderProductPath', lidar_helper_path+'.inputs:renderProductPath')

    controller.attribute(create_render_path+'.inputs:cameraPrim').set([usdrt.Sdf.Path(lidar_path)])
    controller.attribute(lidar_helper_path+'.inputs:frameId').set(lidarFrameId)
    controller.attribute(lidar_helper_path+'.inputs:topicName').set(lidarTopicName)
    controller.attribute(lidar_helper_path+'.inputs:type').set(lidarType)


def add_imu(graph, imu_path, imuFrameId, imuTopicName):
     graph_path = graph.get_path_to_graph()
     tick_node_path = graph_path + '/OnPlaybackTick'
     ros_clock_node_path = graph_path + '/ReadSimTime'
     read_imu_path = graph_path + '/ReadIMU'
     pub_imu_path = graph_path + '/PublishIMU'

     controller = og.Controller
     controller.create_node(read_imu_path, "omni.isaac.sensor.IsaacReadIMU")
     controller.create_node(pub_imu_path, "omni.isaac.ros_bridge.ROS1PublishImu")

     controller.connect(tick_node_path+'.outputs:tick', read_imu_path+'.inputs:execIn')
     controller.connect(ros_clock_node_path+'.outputs:simulationTime', pub_imu_path+'.inputs:timeStamp')
     controller.connect(read_imu_path+'.outputs:execOut', pub_imu_path+'.inputs:execIn')
     controller.connect(read_imu_path+'.outputs:linAcc', pub_imu_path+'.inputs:linearAcceleration')
     controller.connect(read_imu_path+'.outputs:angVel', pub_imu_path+'.inputs:angularVelocity')
     controller.connect(read_imu_path+'.outputs:orientation', pub_imu_path+'.inputs:orientation')

     controller.attribute(read_imu_path+'.inputs:imuPrim').set([usdrt.Sdf.Path(imu_path)])
     controller.attribute(pub_imu_path+'.inputs:frameId').set(imuFrameId)
     controller.attribute(pub_imu_path+'.inputs:topicName').set(imuTopicName)

def add_diff_teleop(graph, robot_path, joint_names:List[str], max_linear_speed, wheel_distance, wheel_radius, topic_name):
     graph_path = graph.get_path_to_graph()
     tick_node_path = graph_path + '/OnPlaybackTick'
     sub_twist_path = graph_path + '/SubscribeTwistDiff'
     scale_path = graph_path + '/ScaleWheel'
     break_ang_path = graph_path + '/Break3_VectorAngVel'
     break_lin_path = graph_path + '/Break3_VectorLinVel'
     diff_contr_path = graph_path + '/DiffController'
     art_contr_path = graph_path + '/ArticulationController'
     
     controller = og.Controller
     controller.create_node(sub_twist_path, "omni.isaac.ros_bridge.ROS1SubscribeTwist")
     controller.create_node(scale_path, "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit")
     controller.create_node(break_ang_path, "omni.graph.nodes.BreakVector3")
     controller.create_node(break_lin_path, "omni.graph.nodes.BreakVector3")
     controller.create_node(diff_contr_path, "omni.isaac.wheeled_robots.DifferentialController")
     controller.create_node(art_contr_path, "omni.isaac.core_nodes.IsaacArticulationController")

     controller.connect(tick_node_path+'.outputs:tick',sub_twist_path+'.inputs:execIn')
     controller.connect(tick_node_path+'.outputs:tick',art_contr_path+'.inputs:execIn')
     controller.connect(sub_twist_path+'.outputs:execOut',diff_contr_path+'.inputs:execIn')
     controller.connect(sub_twist_path+'.outputs:angularVelocity',break_ang_path+'.inputs:tuple')
     controller.connect(sub_twist_path+'.outputs:linearVelocity',scale_path+'.inputs:value')
     controller.connect(scale_path+'.outputs:result',break_lin_path+'.inputs:tuple')
     controller.connect(break_ang_path+'.outputs:z',diff_contr_path+'.inputs:angularVelocity')
     controller.connect(break_lin_path+'.outputs:x',diff_contr_path+'.inputs:linearVelocity')
     controller.connect(diff_contr_path+'.outputs:velocityCommand',art_contr_path+'.inputs:velocityCommand')

     controller.attribute(sub_twist_path+'.inputs:topicName').set(topic_name)
     controller.attribute(diff_contr_path+'.inputs:maxLinearSpeed').set(max_linear_speed)
     controller.attribute(diff_contr_path+'.inputs:wheelDistance').set(wheel_distance)
     controller.attribute(diff_contr_path+'.inputs:wheelRadius').set(wheel_radius)
     controller.attribute(art_contr_path+'.inputs:usePath').set(True)
     controller.attribute(art_contr_path+'.inputs:robotPath').set(robot_path)
     controller.attribute(art_contr_path+'.inputs:jointNames').set(joint_names)


def add_odometry(graph, ogn_node_name, chassis_prim_path, chassis_frame_id, odom_frame_id, odom_topic_name):
     graph_path = graph.get_path_to_graph()
     tick_node_path = graph_path + '/OnPlaybackTick'
     ros_clock_node_path = graph_path + '/ReadSimTime'
     compute_odom_path = graph_path + '/' + ogn_node_name
     pub_odom_path = graph_path + '/Pub' + ogn_node_name
     pub_raw_tf_path = graph_path + '/' + 'PublishRawTFTree'

     controller = og.Controller
     controller.create_node(compute_odom_path, "omni.isaac.core_nodes.IsaacComputeOdometry") 
     controller.create_node(pub_odom_path, "omni.isaac.ros_bridge.ROS1PublishOdometry")
     controller.create_node(pub_raw_tf_path, "omni.isaac.ros_bridge.ROS1PublishRawTransformTree")

     controller.connect(tick_node_path+'.outputs:tick',compute_odom_path+'.inputs:execIn')
     controller.connect(tick_node_path+'.outputs:tick',pub_odom_path+'.inputs:execIn')
     controller.connect(ros_clock_node_path+'.outputs:simulationTime',pub_odom_path+'.inputs:timeStamp')
     controller.connect(compute_odom_path+'.outputs:execOut',pub_odom_path+'.inputs:execIn')
     controller.connect(compute_odom_path+'.outputs:angularVelocity',pub_odom_path+'.inputs:angularVelocity')
     controller.connect(compute_odom_path+'.outputs:linearVelocity',pub_odom_path+'.inputs:linearVelocity')
     controller.connect(compute_odom_path+'.outputs:orientation',pub_odom_path+'.inputs:orientation')
     controller.connect(compute_odom_path+'.outputs:position',pub_odom_path+'.inputs:position')
     controller.connect(compute_odom_path+'.outputs:execOut',pub_raw_tf_path+'.inputs:execIn')
     controller.connect(compute_odom_path+'.outputs:orientation',pub_raw_tf_path+'.inputs:rotation')
     controller.connect(compute_odom_path+'.outputs:position',pub_raw_tf_path+'.inputs:translation')
     controller.connect(ros_clock_node_path+'.outputs:simulationTime',pub_raw_tf_path+'.inputs:timeStamp')

     controller.attribute(compute_odom_path+'.inputs:chassisPrim').set(chassis_prim_path)
     controller.attribute(pub_odom_path+'.inputs:chassisFrameId').set(chassis_frame_id)
     controller.attribute(pub_odom_path+'.inputs:odomFrameId').set(odom_frame_id)
     controller.attribute(pub_odom_path+'.inputs:topicName').set(odom_topic_name)
     controller.attribute(pub_raw_tf_path+'.inputs:childFrameId').set(chassis_frame_id)
     controller.attribute(pub_raw_tf_path+'.inputs:parentFrameId').set(odom_frame_id)
     controller.attribute(pub_raw_tf_path+'.inputs:topicName').set("tf")

def add_tf(graph, ogn_node_name, target_prims: List[str], topic_name = "tf", parent_prim: Optional[str] = None, namespace: Optional[str] = None):
     graph_path = graph.get_path_to_graph()
     tick_node_path = graph_path + '/OnPlaybackTick'
     ros_clock_node_path = graph_path + '/ReadSimTime'
     publish_tf_path = graph_path + '/' + ogn_node_name

     controller = og.Controller
     controller.create_node(publish_tf_path, "omni.isaac.ros_bridge.ROS1PublishTransformTree")

     controller.connect(tick_node_path+'.outputs:tick',publish_tf_path+'.inputs:execIn')
     controller.connect(ros_clock_node_path+'.outputs:simulationTime', publish_tf_path+'.inputs:timeStamp')

     controller.attribute(publish_tf_path+'.inputs:targetPrims').set(target_prims)
     if(namespace is not None):
          controller.attribute(publish_tf_path+'.inputs:nodeNamespace').set(namespace)
     if(parent_prim is not None):
          controller.attribute(publish_tf_path+'.inputs:parentPrim').set(parent_prim)

