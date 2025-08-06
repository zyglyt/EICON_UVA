"""

* 2025无人系统具身智能算法挑战赛 专用代码
* 版权所有 (c) 2025 无人系统具身智能算法挑战赛组委会
* 
* 本源码仅限本赛事参赛团队在比赛过程中使用，
* 禁止任何形式的商业用途、非授权传播或用于其他非比赛场景。
* 
* 依照 GNU 通用公共许可证（GPL）条款授权：
* 参赛者可基于赛事目的对源码进行修改和扩展，
* 但修改后的代码仍受限于本声明的约束条款。
* 
* 本源码按"现状"提供，组委会不承担任何明示或暗示的担保责任，
* 包括但不限于适销性或特定用途适用性的保证。

"""
import os
import sys
import carb
import omni
import random
import numpy as np
from isaacsim import SimulationApp
from scipy.spatial.transform import Rotation 

CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "hide_ui": False,"renderer": "RayTracedLighting"}
kit = SimulationApp(launch_config=CONFIG)

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros1.bridge")
kit.update()

import omni.graph.core as og
import usdrt.Sdf

import rospy
from std_msgs.msg import Header
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from jaka_env import JakaController,JakaRmpFlowController

JAKA_STAGE_PATH = "/World/RobotArm/JAKA_C5_With_DH_PGI/Link_00"

def add_ros1_camera(render_product_path, graph_path, rgb_topic,info_topic,depth_topic):
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("cameraHelperRgb", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("cameraHelperInfo", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("cameraHelperDepth", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("PublishTF", "isaacsim.ros1.bridge.ROS1PublishTransformTree"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
                ("OnTick.outputs:tick", "cameraHelperInfo.inputs:execIn"),
                ("OnTick.outputs:tick", "cameraHelperDepth.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("cameraHelperRgb.inputs:renderProductPath", render_product_path),
                ("cameraHelperRgb.inputs:frameId", "base_link"),
                ("cameraHelperRgb.inputs:topicName", rgb_topic),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("cameraHelperInfo.inputs:renderProductPath", render_product_path),
                ("cameraHelperInfo.inputs:frameId", "base_link"),
                ("cameraHelperInfo.inputs:topicName", info_topic),
                ("cameraHelperInfo.inputs:type", "camera_info"),
                ("cameraHelperDepth.inputs:renderProductPath", render_product_path),
                ("cameraHelperDepth.inputs:frameId", "base_link"),
                ("cameraHelperDepth.inputs:topicName",depth_topic),
                ("cameraHelperDepth.inputs:type", "depth"),
                ("PublishTF.inputs:topicName", "tf"),
                ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path("/World/Cameras/top")]),
            ],
        },
    )
    return ros_camera_graph

g_robotArm_ee_position = np.zeros(3)
g_robotArm_ee_oriention = np.zeros(4)
g_gripper_value = 0.0

g_robot_arm_joint_positions = np.zeros(8)

def robotArm__js_callback(data:JointState):
    global g_robot_arm_joint_positions
    g_robot_arm_joint_positions = np.array(data.position)
    print(f'positions:{g_robot_arm_joint_positions}')

class SimEnvironment:
    def __init__(self,kit:SimulationApp):
        self.kit = kit
        self.current_path = os.getcwd()
        self.ee_pose_pub = None
        self.jointstate_pub = None
        self.gripper_value_pub = None

        from omni.isaac.core.utils.stage import is_stage_loading
        scene_path=self.current_path + "/Content/JAKA/scene_checkerboard.usd"
        omni.usd.get_context().open_stage(scene_path)
        self.kit.update()
        self.kit.update()
        print("Loading stage...")
        while is_stage_loading():
            self.kit.update()
        print("Loading Complete")

        rospy.init_node("isaacsim_jaka", anonymous=True)
        self.ee_pose_pub = rospy.Publisher("Jaka/get_end_effector_pose", PoseStamped,queue_size=10)
        self.jointstate_pub = rospy.Publisher("Jaka/get_jointstate", JointState,queue_size=10)
        self.jointstate_sub = rospy.Subscriber("Jaka/set_jointstate", JointState, robotArm__js_callback)
        from omni.isaac.core import SimulationContext
        self.simulation_context = SimulationContext(stage_units_in_meters=1.0)
        self.simulation_context.initialize_physics()

        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishTF", "isaacsim.ros1.bridge.ROS1PublishTransformTree"),
                    ("PublishClock", "isaacsim.ros1.bridge.ROS1PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishClock.inputs:topicName", "Jaka/clock"),
                    ("PublishTF.inputs:topicName", "tf"),
                    ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(JAKA_STAGE_PATH)]),
                ],
            },
        )

        from omni.kit.viewport.utility import create_viewport_window
        top_viewport = create_viewport_window(viewport_name='camera_top',camera_path='/World/Cameras/top',width=320,height=240,position_x=0, position_y=0)
        ros_camera_graph = add_ros1_camera(top_viewport.viewport_api.get_render_product_path(),"/top_camera","Jaka/camera_rgb","Jaka/camera_info","Jaka/camera_depth")
        og.Controller.evaluate_sync(ros_camera_graph)
        self.kit.update()
        viewport_api = top_viewport.viewport_api
        if viewport_api is not None:
            import omni.syntheticdata._syntheticdata as sd
            rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
            rgb_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv_rgb + "IsaacSimulationGate", viewport_api.get_render_product_path())
            rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
            depth_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv_depth + "IsaacSimulationGate", viewport_api.get_render_product_path())
            camera_info_gate_path = omni.syntheticdata.SyntheticData._get_node_path("PostProcessDispatch" + "IsaacSimulationGate", viewport_api.get_render_product_path())
            og.Controller.attribute(rgb_camera_gate_path + ".inputs:step").set(5)
            og.Controller.attribute(depth_camera_gate_path + ".inputs:step").set(30)
            og.Controller.attribute(camera_info_gate_path + ".inputs:step").set(5)

        yaml_path=self.current_path + "/Content/JAKA/robot/JAKA_C5_With_DH_PGI.yaml"
        urdf_path=self.current_path + "/Content/JAKA/robot/JAKA_C5_With_DH_PGI.urdf"
        #rmpflow_config_path=self.current_path + "/Content/JAKA/robot/JAKA_C5_With_DH_PGI_rmpflow.yaml"
        self.robotArmController = JakaController(robot_arm_prim_path="/World/RobotArm/JAKA_C5_With_DH_PGI",
                                            robot_arm_yaml_path=yaml_path,
                                            robot_arm_urdf_path=urdf_path,
                                            #robot_arm_rmpfloaw_config_path = rmpflow_config_path,
                                            end_effector_name='gripper_center')

    def create_joint_state(self,name, position, velocity=[], effort=[]):
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = name
        js.position = position
        js.velocity = velocity
        js.effort = effort
        return js

    def create_pose_stamped_msg(self, ee_position,ee_euler, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = ee_position[0]
        pose.pose.position.y = ee_position[1]
        pose.pose.position.z = ee_position[2]
        pose.pose.orientation.w = ee_euler[3]
        pose.pose.orientation.x = ee_euler[0]
        pose.pose.orientation.y = ee_euler[1]
        pose.pose.orientation.z = ee_euler[2]
        return pose

    def physics_dt(self):
        return self.simulation_context.get_physics_dt()
    
    def reset(self):
        self.robotArmController.reset()
        arm_jointstate = self.robotArmController.get_joints_state()
        global g_robot_arm_joint_positions
        g_robot_arm_joint_positions = arm_jointstate.positions

    def play(self):
        self.simulation_context.play()
        for _ in range(10):
            self.simulation_context.step(render=True)
            self.kit.update()
        
    def step(self):
        if self.kit.is_running():

            if self.ee_pose_pub:
                position,euler = self.robotArmController.get_end_effector_pose()
                ee_pose_msg = self.create_pose_stamped_msg(position,euler,'map')
                self.ee_pose_pub.publish(ee_pose_msg)

            if self.jointstate_pub:
                arm_jointstate = self.robotArmController.get_joints_state()
                jointstate_msg = self.create_joint_state(name=self.robotArmController._articulation.dof_names,position=arm_jointstate.positions,velocity=arm_jointstate.velocities,effort=arm_jointstate.efforts)
                self.jointstate_pub.publish(jointstate_msg)

            if g_robot_arm_joint_positions.size == 8:
                self.robotArmController.set_joints_state(positions=g_robot_arm_joint_positions,velocities=np.zeros(8),efforts=np.zeros(8))
            self.simulation_context.step(render=True)
            self.kit.update()

    def close(self):
        self.ee_pose_pub.unregister()
        self.jointstate_pub.unregister()
        self.jointstate_sub.unregister()
        self.simulation_context.stop()
        self.kit.close()

def main():
    env = SimEnvironment(kit=kit)
    env.play()
    is_fitst = True
    while env.kit.is_running():
        if is_fitst:
            env.reset()
            is_fitst = False
        env.step()
    env.close()
if __name__ == '__main__':
    main()