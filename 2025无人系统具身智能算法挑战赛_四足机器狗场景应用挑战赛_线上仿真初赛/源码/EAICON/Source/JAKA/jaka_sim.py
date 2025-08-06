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
import threading
from isaacsim import SimulationApp
from scipy.spatial.transform import Rotation 

CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "hide_ui": True,"renderer": "RayTracedLighting"}
kit = SimulationApp(launch_config=CONFIG)

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros1.bridge")
kit.update()
from omni.isaac.core import World
import omni.graph.core as og
import usdrt.Sdf

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Header
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from jaka_env import JakaRmpFlowController

JAKA_STAGE_PATH = "/World/RobotArm/JAKA_C5_With_DH_PGI/base_link"

def add_ros1_camera(render_product_path, graph_path, rgb_topic,info_topic,depth_topic):
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("cameraHelperRgb", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("cameraHelperInfo", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("cameraHelperDepth", "isaacsim.ros1.bridge.ROS1CameraHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
                ("OnTick.outputs:tick", "cameraHelperInfo.inputs:execIn"),
                ("OnTick.outputs:tick", "cameraHelperDepth.inputs:execIn"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("cameraHelperRgb.inputs:renderProductPath", render_product_path),
                ("cameraHelperRgb.inputs:frameId", "camera_top_link"),
                ("cameraHelperRgb.inputs:topicName", rgb_topic),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("cameraHelperInfo.inputs:renderProductPath", render_product_path),
                ("cameraHelperInfo.inputs:frameId", "camera_top_link"),
                ("cameraHelperInfo.inputs:topicName", info_topic),
                ("cameraHelperInfo.inputs:type", "camera_info"),
                ("cameraHelperDepth.inputs:renderProductPath", render_product_path),
                ("cameraHelperDepth.inputs:frameId", "camera_top_link"),
                ("cameraHelperDepth.inputs:topicName",depth_topic),
                ("cameraHelperDepth.inputs:type", "depth"),
            ],
        },   )
    return ros_camera_graph

g_data_lock = threading.Lock()
g_robotArm_ee_position = np.zeros(3)
g_robotArm_ee_orientation = np.array([1.0, 0.0, 0.0, 0.0])
g_gripper_value = 0.001
g_new_goal_received = False

def end_effector_pose_callback(data:PoseStamped):
    global g_robotArm_ee_position, g_robotArm_ee_orientation, g_new_goal_received
    with g_data_lock:
        g_robotArm_ee_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        g_robotArm_ee_orientation = np.array([
            data.pose.orientation.w, 
            data.pose.orientation.x, 
            data.pose.orientation.y, 
            data.pose.orientation.z
        ])
        g_new_goal_received = True
        carb.log_info("接收到新的末端目标位姿。")

def gripper_value_callback(data:Float32):
    global g_gripper_value
    with g_data_lock:
        g_gripper_value = np.clip(data.data, 0.001, 0.04)

class SimEnvironment:
    def __init__(self, kit:SimulationApp):
        self.kit = kit
        self.current_path = os.getcwd()
        self.ee_pose_pub = None
        self.jointstate_pub = None
        
        from omni.isaac.core.utils.stage import is_stage_loading
        scene_path = self.current_path + "/Content/JAKA/scene.usd"
        omni.usd.get_context().open_stage(scene_path)
        self.kit.update()
        print("正在加载场景...")
        while is_stage_loading():
            self.kit.update()
        print("场景加载完成。")

        self.stage = omni.usd.get_context().get_stage()

        rospy.init_node("isaacsim_jaka", anonymous=True)
        self.ee_pose_sub = rospy.Subscriber("Jaka/set_end_effector_pose", PoseStamped, end_effector_pose_callback)
        self.gripper_value_sub = rospy.Subscriber("Jaka/set_gripper_value", Float32, gripper_value_callback)
        self.ee_pose_pub = rospy.Publisher("Jaka/get_end_effector_pose", PoseStamped, queue_size=10)
        self.jointstate_pub = rospy.Publisher("Jaka/get_jointstate", JointState, queue_size=10)
        self.gripper_efforts_pub = rospy.Publisher("Jaka/get_gripper_efforts", Float32MultiArray, queue_size=10)
        self.gripper_is_captured_pub = rospy.Publisher("Jaka/gripper_is_captured", Bool, queue_size=10)
        
        self.world = World(stage_units_in_meters=1.0)

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
                    ("PublishClock.inputs:topicName", "/clock"),
                    ("PublishTF.inputs:topicName", "/tf"),
                    ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(JAKA_STAGE_PATH)]),
                ],
            },
        )

        from omni.kit.viewport.utility import create_viewport_window
        top_viewport = create_viewport_window(viewport_name='camera_top',camera_path='/World/Cameras/top',width=320,height=240,position_x=10, position_y=15)
        
        ros_camera_graph = add_ros1_camera(
            top_viewport.viewport_api.get_render_product_path(),
            "/top_camera_graph",
            "Jaka/camera/rgb",
            "Jaka/camera/camera_info",
            "Jaka/camera/depth"
        )
        
        self.robotArmController = JakaRmpFlowController(
            stage=self.stage,
            robot_arm_prim_path="/World/RobotArm/JAKA_C5_With_DH_PGI",
            robot_arm_yaml_path=os.path.join(
                self.current_path, "Content/JAKA/robot/JAKA_C5_With_DH_PGI.yaml"
            ),
            robot_arm_urdf_path=os.path.join(
                self.current_path, "Content/JAKA/robot/JAKA_C5_With_DH_PGI.urdf"
            ),
            robot_arm_rmpflow_config_path=os.path.join(
                self.current_path, "Content/JAKA/robot/JAKA_C5_With_DH_PGI_rmpflow.yaml"
            ),
            end_effector_name="gripper_center",
            gripper_joint_names=[
                "/World/RobotArm/JAKA_C5_With_DH_PGI/base_link/finger1_joint",
                "/World/RobotArm/JAKA_C5_With_DH_PGI/base_link/finger2_joint",
            ],
            gripper_collisions_names=[
                "/World/RobotArm/JAKA_C5_With_DH_PGI/gripper_center/convexHull"
            ],
        )

    
        robot_arm_articulation = self.robotArmController.get_articulation()
        self.world.scene.add(robot_arm_articulation)

        self.gripper_is_captured_pub = rospy.Publisher(
        "Jaka/gripper_is_captured",   
        Bool,                         
        queue_size=1,
        latch=True)     
        
    def play(self):
        self.reset()
        self.world.play()
        for _ in range(10):
            self.world.step(render=True)

    def reset(self):
        self.world.reset()
        self.robotArmController.reset()

        with g_data_lock:
            global g_robotArm_ee_position, g_robotArm_ee_orientation, g_new_goal_received
            position, orientation = self.robotArmController.get_end_effector_pose()
            g_robotArm_ee_position = position
            g_robotArm_ee_orientation = orientation
            g_new_goal_received = False
            
    def step(self):
        global g_new_goal_received, g_robotArm_ee_position, g_robotArm_ee_orientation, g_gripper_value
        
        local_pos_goal, local_rot_goal = None, None
        should_plan = False
        
        with g_data_lock:
            if g_new_goal_received and not self.robotArmController._tracking_in_progress:
                local_pos_goal = g_robotArm_ee_position.copy()
                local_rot_goal = g_robotArm_ee_orientation.copy()
                g_new_goal_received = False
                should_plan = True

        if should_plan:
            try:
                self.robotArmController.plan_and_execute_trajectory(local_pos_goal, local_rot_goal)
            except Exception as e:
                carb.log_error(f"路径规划执行出错: {e}")

        with g_data_lock:
            local_gripper_val = g_gripper_value
        
        self.robotArmController.forward_and_track(local_gripper_val)
        self.robotArmController.forward_and_track(local_gripper_val)

        bool_msg = Bool()
        bool_msg.data = self.robotArmController.gripper_is_close
        self.gripper_is_captured_pub.publish(bool_msg)


        self.publish_ros_data()
        self.world.step(render=True)

        self.publish_ros_data()
        self.world.step(render=True)
    
    def publish_ros_data(self):
        pos, rot_wxyz = self.robotArmController.get_end_effector_pose()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = pos
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = \
            rot_wxyz[1], rot_wxyz[2], rot_wxyz[3], rot_wxyz[0]
        self.ee_pose_pub.publish(pose_msg)

        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        js_msg.name  = self.robotArmController._articulation.dof_names

        state = self.robotArmController._articulation.get_joints_state()
        if state and state.positions is not None:
            js_msg.position = state.positions.tolist()
            js_msg.velocity = state.velocities.tolist() if state.velocities is not None else []
            js_msg.effort   = state.efforts.tolist()   if state.efforts   is not None else []
        else:
            pos = self.robotArmController._articulation.get_joint_positions()
            vel = self.robotArmController._articulation.get_joint_velocities()
            js_msg.position = pos.tolist() if pos is not None else []
            js_msg.velocity = vel.tolist() if vel is not None else []
            js_msg.effort   = []

        self.jointstate_pub.publish(js_msg)


    def close(self):
        self.world.stop()
        rospy.signal_shutdown("Isaac Sim is shutting down")
        self.kit.close()

def main():
    env = SimEnvironment(kit=kit)
    
    try:
        env.play()
        while kit.is_running():
            env.step()
    except Exception as e:
        import traceback
        print(f"仿真主循环异常: {e}")
        traceback.print_exc()
    finally:
        env.close()

if __name__ == '__main__':
    main()
