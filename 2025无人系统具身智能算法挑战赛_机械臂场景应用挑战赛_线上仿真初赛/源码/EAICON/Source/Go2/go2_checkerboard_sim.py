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
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import threading
import numpy as np
import math
import carb
from isaacsim import SimulationApp
from scipy.spatial.transform import Rotation as R

CONFIG = {
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    "headless": False,
    "hide_ui": True,
    "renderer": "RayTracedLighting",
}
kit = SimulationApp(launch_config=CONFIG)

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros1.bridge")
kit.update()

import omni.kit.viewport.utility
import omni.replicator.core as rep
import omni.graph.core as og
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.sensors.rtx import LidarRtx
from omni.isaac.core.prims import XFormPrim
from go2_env import Go2FlatTerrainPolicy

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, TransformStamped

data_lock = threading.Lock()

g_robot_arm_joint_positions = np.zeros(8)

def robotArm__js_callback(data: JointState):

    global g_robot_arm_joint_positions
    with data_lock:
        g_robot_arm_joint_positions = np.array(data.position, dtype=np.float32)
        robot_state.ee_command_received = True

class RobotState:

    def __init__(self):
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_yaw = 0.0
        self.ee_position = np.zeros(3)
        self.ee_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.gripper_value = 0.001
        self.ee_command_received = False

robot_state = RobotState()

def robot_cmd_vel_x_callback(cmd: Float32):
    with data_lock:
        robot_state.cmd_vel_x = cmd.data

def robot_cmd_vel_y_callback(cmd: Float32):
    with data_lock:
        robot_state.cmd_vel_y = cmd.data

def robot_cmd_vel_yaw_callback(cmd: Float32):
    with data_lock:
        robot_state.cmd_vel_yaw = cmd.data

class SimEnvironment:
    def __init__(self):

        self.current_path = os.getcwd()
        self.first_step = True
        self.prev_pos = None
        self.prev_yaw = None
        self.prev_time = None
        self.odom_origin = None
        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        self.vx = self.vy = self.vyaw = 0.0

        self.world = None
        self.go2 = None
        self.lidar = None
        self.imu_sensor = None
        self.global_camera = None
        self.front_camera = None
        self.wrist_camera = None
        self.persp_camera = None

        self.ros_initialized = False
        self.ros_pubs = {}

        vp = omni.kit.viewport.utility.get_active_viewport()
        carb.settings.get_settings().set_bool(
            f"/persistent/app/viewport/{vp.id}/fillViewport", True
        )

    def load(self):
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=1/500,
            rendering_dt=1/50
        )

        ground = define_prim("/World/Ground", "Xform")
        ground.GetReferences().AddReference(
            os.path.join(self.current_path, "Content/Go2/scene/Ground.usd")
        )

        self.go2 = Go2FlatTerrainPolicy(
            prim_path="/World/Go2",
            name="Go2",
            position=np.array([3.61487, 4.2607, 0.3]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        )

        checker = define_prim("/World/checkerboard", "Xform")
        checker.GetReferences().AddReference(
            os.path.join(self.current_path, "Content/Go2/checkerboard/checkerboard_7x9.usdc")
        )
        cb = XFormPrim(prim_path="/World/checkerboard")
        cb.set_world_pose(
            position=np.array([3.61487, 3.8607, 0.02]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0])
        )

        self._setup_sensors()

    def _setup_sensors(self):
        self.lidar = self.world.scene.add(
            LidarRtx(
                prim_path="/World/Go2/base/lidar_sensor",
                name="lidar",
                rotation_frequency=200,
                pulse_time=1,
                translation=(0.0, 0.0, 0.4),
                orientation=(1.0, 0.0, 0.0, 0.0),
                config_file_name="L1",
            )
        )

        from omni.isaac.sensor import Camera
        from omni.kit.viewport.utility import create_viewport_window

        self.global_camera = Camera(
            prim_path="/World/Ground/GlobalCamera",
            resolution=(2048, 2048)
        )
        self.global_camera.initialize()

        self.front_camera = Camera(
            prim_path="/World/Go2/base/frontCamera",
            translation=(0.0, 0.0, 0.5),
            orientation=(1.0, 0.0, 0.0, 0.0),
            resolution=(640, 480)
        )
        self.front_camera.initialize()
        self.front_camera.set_focal_length(4)
        pos, ori = self.front_camera.get_world_pose()
        tilt = R.from_euler('Y', math.radians(40.0)).as_quat()
        r_orig = R.from_quat([ori[1], ori[2], ori[3], ori[0]])
        r_new = r_orig * R.from_quat(tilt)
        nq = r_new.as_quat()
        self.front_camera.set_world_pose(
            position=pos,
            orientation=[nq[3], nq[0], nq[1], nq[2]]
        )

        self.wrist_camera = Camera(
            prim_path="/World/Go2/Link6/wristCamera",
            resolution=(640, 480)
        )
        self.wrist_camera.initialize()
        pos_w, ori_w = self.wrist_camera.get_world_pose()
        pos_w[2] += 0.03
        self.wrist_camera.set_world_pose(position=pos_w, orientation=ori_w)

        self.persp_camera = Camera(prim_path="/OmniverseKit_Persp")
        self.persp_camera.set_focal_length(8)

        create_viewport_window(
            viewport_name="camera_top",
            camera_path="/World/Go2/base/frontCamera",
            width=320, height=240, position_x=10, position_y=15
        )
        create_viewport_window(
            viewport_name="camera_wrist",
            camera_path="/World/Go2/Link6/wristCamera",
            width=320, height=240, position_x=1110, position_y=15
        )

        from isaacsim.sensors.physics import IMUSensor
        self.imu_sensor = IMUSensor(
            prim_path="/World/Go2/base/imu_sensor",
            name="imu",
            frequency=60,
            translation=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
            linear_acceleration_filter_size=10,
            angular_velocity_filter_size=10,
            orientation_filter_size=10,
        )

    def init_ros(self):
        kit.update()
        rospy.init_node("isaacsim_go2", anonymous=True)
        rospy.set_param("/use_sim_time", True)
        self._setup_ros_clock()
        self._init_ros_pubsub()
        self._setup_ros_bridges()
        self.ros_initialized = True

    def _setup_ros_clock(self):
        og.Controller.edit(
            {"graph_path": "/ROS_Clock", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishClock", "isaacsim.ros1.bridge.ROS1PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
            },
        )

    def _init_ros_pubsub(self):
        self.ros_pubs = {
            "tf": rospy.Publisher("tf", TFMessage, queue_size=10),
            "odom": rospy.Publisher("odom", Odometry, queue_size=10),
            "imu": rospy.Publisher("imu/data", Imu, queue_size=10),
            "joints": rospy.Publisher("joint_states", JointState, queue_size=10),
            "ee_pose": rospy.Publisher("end_effector/pose", PoseStamped, queue_size=10),
        }

        rospy.Subscriber("cmd_vel_x", Float32, robot_cmd_vel_x_callback)
        rospy.Subscriber("cmd_vel_y", Float32, robot_cmd_vel_y_callback)
        rospy.Subscriber("cmd_vel_yaw", Float32, robot_cmd_vel_yaw_callback)
        rospy.Subscriber("end_effector/target_pose", PoseStamped, self._target_pose_callback)
        rospy.Subscriber("gripper/target", Float32, self._gripper_value_callback)
        rospy.Subscriber("set_jointstate", JointState, robotArm__js_callback)

    def _target_pose_callback(self, data: PoseStamped):
        base_p, base_q = self.go2.robot.get_world_pose()
        Rb = R.from_quat([base_q[1], base_q[2], base_q[3], base_q[0]])
        rel_p = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        world_p = np.array(base_p) + Rb.apply(rel_p)
        rel_q = np.array([data.pose.orientation.w, data.pose.orientation.x,
                          data.pose.orientation.y, data.pose.orientation.z])
        Rrel = R.from_quat([rel_q[1], rel_q[2], rel_q[3], rel_q[0]])
        Rw = Rb * Rrel
        qw = Rw.as_quat()
        world_q = np.array([qw[3], qw[0], qw[1], qw[2]])
        with data_lock:
            robot_state.ee_position = world_p
            robot_state.ee_orientation = world_q
            robot_state.ee_command_received = True

    def _gripper_value_callback(self, cmd: Float32):
        with data_lock:
            robot_state.gripper_value = np.clip(cmd.data, 0.001, 0.03)

    def _setup_ros_bridges(self):
        kit.update()
        og.Controller.edit(
            {"graph_path": "/World/globalCameraGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("cameraRgb", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "cameraRgb.inputs:execIn")
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("cameraRgb.inputs:renderProductPath", self.global_camera.get_render_product_path()),
                    ("cameraRgb.inputs:frameId", "odom"),
                    ("cameraRgb.inputs:topicName", "camera/global"),
                    ("cameraRgb.inputs:type", "rgb"),
                ],
            },
        )
        og.Controller.edit(
            {"graph_path": "/World/frontCameraGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("cameraRgb", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "cameraRgb.inputs:execIn")
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("cameraRgb.inputs:renderProductPath", self.front_camera.get_render_product_path()),
                    ("cameraRgb.inputs:frameId", "odom"),
                    ("cameraRgb.inputs:topicName", "camera/front"),
                    ("cameraRgb.inputs:type", "rgb"),
                ],
            },
        )
        og.Controller.edit(
            {"graph_path": "/World/wristCameraGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("cameraRgb", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                    ("cameraInfo", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                    ("cameraDepth", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                    ("PublishTF", "isaacsim.ros1.bridge.ROS1PublishTransformTree"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "cameraRgb.inputs:execIn"),
                    ("OnTick.outputs:tick", "cameraInfo.inputs:execIn"),
                    ("OnTick.outputs:tick", "cameraDepth.inputs:execIn"),
                    ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("cameraRgb.inputs:renderProductPath", self.wrist_camera.get_render_product_path()),
                    ("cameraRgb.inputs:frameId", "base_link"),
                    ("cameraRgb.inputs:topicName", "camera/wrist/rgb"),
                    ("cameraRgb.inputs:type", "rgb"),
                    ("cameraInfo.inputs:renderProductPath", self.wrist_camera.get_render_product_path()),
                    ("cameraInfo.inputs:frameId", "base_link"),
                    ("cameraInfo.inputs:topicName", "camera/wrist/info"),
                    ("cameraInfo.inputs:type", "camera_info"),
                    ("cameraDepth.inputs:renderProductPath", self.wrist_camera.get_render_product_path()),
                    ("cameraDepth.inputs:frameId", "base_link"),
                    ("cameraDepth.inputs:topicName", "camera/wrist/depth"),
                    ("cameraDepth.inputs:type", "depth"),
                    ("PublishTF.inputs:topicName", "tf"),
                    ("PublishTF.inputs:targetPrims", ["/World/Go2/Link6/wristCamera"]),
                ],
            },
        )
        writer_pc = rep.writers.get("RtxLidarROS1PublishPointCloud")
        writer_pc.initialize(topicName="lidar/points", frameId="lidar")
        writer_pc.attach([self.lidar.get_render_product_path()])
        writer_scan = rep.writers.get("RtxLidarROS1PublishLaserScan")
        writer_scan.initialize(topicName="scan", frameId="lidar")
        writer_scan.attach([self.lidar.get_render_product_path()])
        kit.update()

    def play(self):
        self.world.reset()
        self.world.add_physics_callback("physics_step", self._physics_step)
        self.world.step(render=True)

    def _physics_step(self, step_size):
        if self.first_step:
            self._initialize_robot()
            return
        self._process_robot_control(step_size)
        self._update_odom()

    def _initialize_robot(self):
        self.go2.initialize()
        self.go2.init_rmpflow(
            gripper_collisions_name="/World/Ground/overlap_mesh",
            gripper_joint_names=[
                "/World/Go2/Link6/Gripper_left",
                "/World/Go2/Link6/Gripper_right",
            ],
        )
        self.go2.robot.set_joints_default_state(self.go2.default_pos)
        self.go2.post_reset()
        pos0, _ = self.go2.robot.get_world_pose()
        self.odom_origin = np.array(pos0, dtype=np.float64)
        self.first_step = False

    def _process_robot_control(self, step_size):
        with data_lock:
            base_cmd = np.array([
                robot_state.cmd_vel_x,
                robot_state.cmd_vel_y,
                robot_state.cmd_vel_yaw
            ], dtype=np.float32)
            ee_new = robot_state.ee_command_received
            robot_state.ee_command_received = False

        self.go2.go2_forward(step_size, base_cmd)

        if ee_new and g_robot_arm_joint_positions.size == 8:
            self.go2.d1_forward_positions(step_size, g_robot_arm_joint_positions)

        left_eff, right_eff = self.go2.get_gripper_efforts()
        if left_eff >= 1.0 and right_eff >= 1.0:
            self.go2.close_gripper()
        else:
            self.go2.open_gripper()

    def _update_odom(self):
        pos_w, quat_w = self.go2.robot.get_world_pose()
        lin_w = self.go2.robot.get_linear_velocity()
        ang_w = self.go2.robot.get_angular_velocity()
        now = rospy.Time.now()

        if self.prev_pos is None:
            self.prev_pos = np.array(pos_w)
            self.prev_yaw = R.from_quat([quat_w[1], quat_w[2], quat_w[3], quat_w[0]]).as_euler('xyz')[2]
            self.prev_time = now
            return

        delta = np.array(pos_w) - self.odom_origin
        x_tmp, y_tmp = delta[0], delta[1]
        yaw_now = R.from_quat([quat_w[1], quat_w[2], quat_w[3], quat_w[0]]).as_euler('xyz')[2]
        yaw_cont = self.prev_yaw + np.unwrap([self.prev_yaw, yaw_now])[1] - self.prev_yaw

        if (np.linalg.norm([x_tmp - self.odom_x, y_tmp - self.odom_y]) > 0.2 or
            abs(yaw_cont - self.prev_yaw) > math.radians(30)):
            self.odom_origin += np.array(pos_w) - self.prev_pos
            x_tmp, y_tmp = self.prev_pos[0] - self.odom_origin[0], self.prev_pos[1] - self.odom_origin[1]
            yaw_cont = self.prev_yaw

        R_wb = np.array([
            [math.cos(yaw_cont), math.sin(yaw_cont)],
            [-math.sin(yaw_cont), math.cos(yaw_cont)],
        ])
        vx_b, vy_b = R_wb @ lin_w[:2]
        self.odom_x, self.odom_y, self.odom_yaw = x_tmp, y_tmp, yaw_cont
        self.vx, self.vy, self.vyaw = vx_b, vy_b, ang_w[2]
        self.prev_pos = np.array(pos_w)
        self.prev_yaw = yaw_cont
        self.prev_time = now

    def step(self):
        kit.update()
        self._publish_pose()
        self._publish_joints()
        self._publish_odom()
        self._publish_imu()
        self._publish_tf()
        self.world.step(render=True)

    def _publish_pose(self):
        ee_p, ee_q = self.go2.get_end_effector_pose()
        base_p, base_q = self.go2.robot.get_world_pose()
        Rb = R.from_quat([base_q[1], base_q[2], base_q[3], base_q[0]])
        Re = R.from_quat([ee_q[1], ee_q[2], ee_q[3], ee_q[0]])
        Rrel = Rb.inv() * Re
        prel = Rb.inv().apply(np.array(ee_p) - np.array(base_p))
        qrel = Rrel.as_quat()
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = prel
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = qrel[3], qrel[0], qrel[1], qrel[2]
        self.ros_pubs["ee_pose"].publish(msg)

    def _publish_joints(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.go2.robot.dof_names
        msg.position = self.go2.robot.get_joint_positions().tolist()
        self.ros_pubs["joints"].publish(msg)

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        msg.pose.pose.position.z = 0.0
        q = R.from_euler('z', self.odom_yaw).as_quat()
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = self.vx, self.vy, self.vyaw
        cov = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]
        msg.pose.covariance = cov
        msg.twist.covariance = cov
        self.ros_pubs["odom"].publish(msg)

    def _publish_imu(self):
        imu = self.imu_sensor.get_current_frame()
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = imu["lin_acc"]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = imu["ang_vel"]
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = imu["orientation"]
        self.ros_pubs["imu"].publish(msg)

    def _publish_tf(self):
        tfm = TFMessage()
        t1 = TransformStamped()
        t1.header.stamp = rospy.Time.now()
        t1.header.frame_id = "odom"
        t1.child_frame_id = "base_link"
        t1.transform.translation.x, t1.transform.translation.y = self.odom_x, self.odom_y
        q = R.from_euler('z', self.odom_yaw).as_quat()
        t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w = q
        tfm.transforms.append(t1)
        t2 = TransformStamped()
        t2.header.stamp = rospy.Time.now()
        t2.header.frame_id = "base_link"
        t2.child_frame_id = "lidar"
        t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z = (0.0, 0.0, 0.4)
        t2.transform.rotation.x, t2.transform.rotation.y, t2.transform.rotation.z, t2.transform.rotation.w = (0.0, 0.0, 0.0, 1.0)
        tfm.transforms.append(t2)
        self.ros_pubs["tf"].publish(tfm)

    def shutdown(self):
        if self.ros_initialized:
            for pub in self.ros_pubs.values():
                pub.unregister()
        kit.close()

def main():
    env = SimEnvironment()
    env.load()
    env.init_ros()
    env.play()
    try:
        while kit.is_running():
            env.step()
    except KeyboardInterrupt:
        pass
    finally:
        env.shutdown()

if __name__ == "__main__":
    main()
