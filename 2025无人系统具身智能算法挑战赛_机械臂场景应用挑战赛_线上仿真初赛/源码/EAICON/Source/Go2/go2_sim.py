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
from scipy.interpolate import CubicSpline
# from omni.isaac.core.utils.types import ArticulationAction

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
# from go2_env import Go2FlatTerrainPolicy
from go2_env import Go2FlatTerrainPolicy, PolicyController, trrt_star_optimized
# from go2_env import Go2FlatTerrainPolicy, PolicyController

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, TransformStamped

data_lock = threading.Lock()

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
        self.odom_origin = None
        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        self.vx = self.vy = self.vyaw = 0.0
        self.prev_pos = None
        self.prev_yaw = None
        self.prev_time = None
        self.first_step = True
        self.world = None
        self.go2 = None
        self.lidar = None
        self.imu_sensor = None
        self.global_camera = None
        self.front_camera = None
        self.wrist_camera = None
        self.persp_camera = None
        self.ros_initialized = False
        self.ros_pubs = {
            "tf": None,
            "odom": None,
            "imu": None,
            "joints": None,
            "ee_pose": None,
        }
        self._tracking_arm = False
        self._traj_start_time = None
        self._traj_positions = None
        self._traj_times = None
        self._traj_total_time = 0.0

        viewport = omni.kit.viewport.utility.get_active_viewport()
        carb.settings.get_settings().set_bool(
            f"/persistent/app/viewport/{viewport.id}/fillViewport", True
        )

    def load(self):
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=1 / 500,
            rendering_dt=1 / 50
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
        self._setup_sensors()

    def _setup_sensors(self):
        # Lidar
        self.lidar = self.world.scene.add(  # type: ignore
            LidarRtx(
                prim_path="/World/Go2/base/lidar_sensor",
                name="lidar",
                rotation_frequency=200,
                pulse_time=1,
                translation=(0.0, 0, 0.4),
                orientation=(1.0, 0.0, 0.0, 0.0),
                config_file_name="L1",
            )
        )

        # Cameras
        from omni.isaac.sensor import Camera
        from omni.kit.viewport.utility import create_viewport_window

        # Global camera
        self.global_camera = Camera(
            prim_path="/World/Ground/GlobalCamera",
            resolution=(2048, 2048)
        )
        self.global_camera.initialize()

        # Front camera
        self.front_camera = Camera(
            prim_path="/World/Go2/base/frontCamera",
            translation=(0.0, 0.0, 0.50),
            orientation=(1.0, 0.0, 0.0, 0.0),
            resolution=(640, 480)
        )
        self.front_camera.initialize()
        self.front_camera.set_focal_length(4)

        pos, ori = self.front_camera.get_world_pose()
        tilt_deg = 40.0
        rot_down = R.from_euler('Y', math.radians(tilt_deg)).as_quat()
        r_orig = R.from_quat([ori[1], ori[2], ori[3], ori[0]])
        r_tilt = R.from_quat(rot_down)
        r_new = r_orig * r_tilt
        new_q = r_new.as_quat()
        self.front_camera.set_world_pose(
            position=pos,
            orientation=[new_q[3], new_q[0], new_q[1], new_q[2]]
        )

        # Wrist camera
        self.wrist_camera = Camera(
            prim_path="/World/Go2/Link6/wristCamera",
            resolution=(640, 480)
        )
        self.wrist_camera.initialize()
        pos, ori = self.wrist_camera.get_world_pose()
        new_pos = pos.copy()
        new_pos[2] += 0.03
        self.wrist_camera.set_world_pose(position=new_pos, orientation=ori)

        # Perspective camera
        self.persp_camera = Camera(prim_path="/OmniverseKit_Persp")
        self.persp_camera.set_focal_length(8)

        # Viewport windows
        create_viewport_window(
            viewport_name="camera_top",
            camera_path="/World/Go2/base/frontCamera",
            width=320, height=240,
            position_x=10, position_y=15,
        )
        create_viewport_window(
            viewport_name="camera_wrist",
            camera_path="/World/Go2/Link6/wristCamera",
            width=320, height=240,
            position_x=1110, position_y=15,
        )

        # IMU
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

    def _target_pose_callback(self, data: PoseStamped):
        base_pos, base_rot = self.go2.robot.get_world_pose()
        R_base = R.from_quat([base_rot[1], base_rot[2], base_rot[3], base_rot[0]])

        rel_pos = np.array([
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z
        ])
        world_pos = np.array(base_pos) + R_base.apply(rel_pos)

        rel_q = np.array([
            data.pose.orientation.w,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z
        ])
        R_rel = R.from_quat([rel_q[1], rel_q[2], rel_q[3], rel_q[0]])
        R_world = R_base * R_rel
        q_xyzw = R_world.as_quat()
        world_q = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])

        with data_lock:
            robot_state.ee_position = world_pos
            robot_state.ee_orientation = world_q
            robot_state.ee_command_received = True

    def _gripper_value_callback(self, cmd: Float32):
        with data_lock:
            robot_state.gripper_value = np.clip(cmd.data, 0.001, 0.03)

    def _setup_ros_bridges(self):
        kit.update()
        # global camera
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
        # front camera
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
        # wrist cameras + TF
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
        # Lidar pointcloud + scan
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
        cmd = np.array([
            robot_state.cmd_vel_x,
            robot_state.cmd_vel_y,
            robot_state.cmd_vel_yaw
        ], dtype=np.float64)

        # ----------- 新增逻辑 -----------
        if np.allclose(cmd, [0.0, 0.0, 0.0]):
            # 没有行走指令 → 维持 / 开启锁腿
            self.go2.freeze_legs = True
        else:
            # 有行走指令 → 先解锁，再正常前进
            if self.go2.freeze_legs:
                # 热身 2‑3 帧；简单起见这里只解锁
                self.go2.freeze_legs = False
        # --------------------------------
        self.go2.go2_forward(step_size, cmd)
        self._process_robot_control(step_size)
        self._update_odom()
        self.go2.set_gripper(robot_state.gripper_value)

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
        pos_init, _ = self.go2.robot.get_world_pose()
        self.odom_origin = np.array(pos_init, dtype=np.float64)
        self.first_step = False
        self.go2.freeze_legs = True

   
    def _process_robot_control(self, step_size):
        with data_lock:
            ee_pos = robot_state.ee_position.copy()
            ee_rot = robot_state.ee_orientation.copy()
            ee_new = robot_state.ee_command_received
            robot_state.ee_command_received = False

        if ee_new and not self._tracking_arm:
            self.go2.freeze_legs = True   
            print("[SimEnv] 收到新末端目标，开始规划机械臂...")
            try:
                q_full = np.array(self.go2.robot.get_joint_positions(), dtype=np.float64)

                self.go2._rmpflow.set_end_effector_target(ee_pos, ee_rot)
                self.go2._rmpflow.update_world()
                bp, bq = self.go2.robot.get_world_pose()
                self.go2._kin_solver.set_robot_base_pose(bp, bq)

                ik_action = self.go2._art_kin.compute_inverse_kinematics(
                    target_position=ee_pos,
                    target_orientation=ee_rot
                )[0]
                if ik_action is None:
                    raise RuntimeError("IK 解算失败，无可行解。")

                arm_indices = ik_action.joint_indices
                q_arm_goal = np.array(ik_action.joint_positions, dtype=np.float64)  
                q_arm_start = q_full[arm_indices].copy()

                view = self.go2.robot._articulation_view
                if hasattr(view, "get_limits"):
                    lower, upper = view.get_limits()            
                elif hasattr(view, "get_joint_limits"):
                    lower, upper = view.get_joint_limits()
                else:
                    D = len(q_full)
                    lower = np.full(D, -math.pi)
                    upper = np.full(D,  math.pi)
                all_limits = np.stack([lower, upper], axis=1)  
                joint_limits6 = all_limits[arm_indices].tolist()  

                path6 = trrt_star_optimized(
                    start=q_arm_start,
                    goal=q_arm_goal,
                    validator=lambda q: True,
                    joint_limits=joint_limits6,
                    max_iter=5000,
                    step_size=0.05,
                    radius=0.2,
                )
                if path6 is None:
                    raise RuntimeError("TRRT* 未找到路径")

                N = len(path6)
                duration = 5.0
                times = np.linspace(0, duration, N)
                cs6 = [CubicSpline(times, [p[d] for p in path6]) for d in range(len(arm_indices))]
                t_query = np.linspace(0, duration, max(100, N * 5))
                traj6 = np.vstack([cs6[d](t_query) for d in range(len(arm_indices))]).T  # (M,6)

                M = traj6.shape[0]
                traj_full = np.tile(q_full, (M, 1))      
                traj_full[:, arm_indices] = traj6        

                self.go2.trajectory = traj_full
                self.go2.time_from_start = t_query
                self.go2.trajectory_duration = duration
                self.go2.trajectory_start_time = None
                self._tracking_arm = True
                print(f"[SimEnv] 规划完成 {M} 点，开始跟踪。")

            except Exception as e:
                print(f"[SimEnv] 机械臂轨迹规划失败: {e}")
                self._tracking_arm = False

        if self._tracking_arm:
            try:
                done = self.go2.track_trajectory()
                if done:
                    self._tracking_arm = False
                    self.go2.freeze_legs = np.allclose(
                        [robot_state.cmd_vel_x,
                        robot_state.cmd_vel_y,
                        robot_state.cmd_vel_yaw],
                        [0.0, 0.0, 0.0]
                    )
                    print("[SimEnv] 机械臂轨迹跟踪完成。")
            except Exception as e:
                print(f"[SimEnv] 跟踪出错: {e}")
                self._tracking_arm = False

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
            self.odom_origin = np.array(pos_w, dtype=np.float64)
            self.prev_pos = np.array(pos_w, dtype=np.float64)
            self.prev_yaw = R.from_quat(
                [quat_w[1], quat_w[2], quat_w[3], quat_w[0]]
            ).as_euler("xyz")[2]
            self.prev_time = now
            return

        delta_p = np.array(pos_w) - self.odom_origin
        x_tmp, y_tmp = delta_p[0], delta_p[1]
        yaw_now = R.from_quat(
            [quat_w[1], quat_w[2], quat_w[3], quat_w[0]]
        ).as_euler("xyz")[2]
        yaw_cont = self.prev_yaw + np.unwrap([self.prev_yaw, yaw_now])[1] - self.prev_yaw

        if (np.linalg.norm([x_tmp - self.odom_x, y_tmp - self.odom_y]) > 0.20
            or abs(yaw_cont - self.prev_yaw) > math.radians(30)
        ):
            self.odom_origin += np.array(pos_w) - self.prev_pos
            x_tmp, y_tmp = (
                self.prev_pos[0] - self.odom_origin[0],
                self.prev_pos[1] - self.odom_origin[1],
            )
            yaw_cont = self.prev_yaw

        R_wb = np.array([
            [math.cos(yaw_cont), math.sin(yaw_cont)],
            [-math.sin(yaw_cont), math.cos(yaw_cont)],
        ])
        vx_b, vy_b = R_wb @ lin_w[:2]
        self.odom_x, self.odom_y, self.odom_yaw = x_tmp, y_tmp, yaw_cont
        self.vx, self.vy, self.vyaw = vx_b, vy_b, ang_w[2]
        self.prev_pos = np.array(pos_w, dtype=np.float64)
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
        ee_pos_w, ee_rot_w = self.go2.get_end_effector_pose()
        base_pos_w, base_rot_w = self.go2.robot.get_world_pose()
        ee_xyzw = [ee_rot_w[1], ee_rot_w[2], ee_rot_w[3], ee_rot_w[0]]
        base_xyzw = [base_rot_w[1], base_rot_w[2], base_rot_w[3], base_rot_w[0]]
        R_base = R.from_quat(base_xyzw)
        R_ee = R.from_quat(ee_xyzw)
        R_rel = R_base.inv() * R_ee
        pos_rel = R_base.inv().apply(np.array(ee_pos_w) - np.array(base_pos_w))
        rel_xyzw = R_rel.as_quat()
        rel_wxyz = [rel_xyzw[3], rel_xyzw[0], rel_xyzw[1], rel_xyzw[2]]
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = pos_rel[0]
        msg.pose.position.y = pos_rel[1]
        msg.pose.position.z = pos_rel[2]
        msg.pose.orientation.w = rel_wxyz[0]
        msg.pose.orientation.x = rel_wxyz[1]
        msg.pose.orientation.y = rel_wxyz[2]
        msg.pose.orientation.z = rel_wxyz[3]
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
        q = R.from_euler("z", self.odom_yaw).as_quat()
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.vyaw
        cov = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]
        msg.pose.covariance = cov
        msg.twist.covariance = cov
        self.ros_pubs["odom"].publish(msg)

    def _publish_imu(self):
        imu_data = self.imu_sensor.get_current_frame()
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"
        msg.linear_acceleration.x = imu_data["lin_acc"][0]
        msg.linear_acceleration.y = imu_data["lin_acc"][1]
        msg.linear_acceleration.z = imu_data["lin_acc"][2]
        msg.angular_velocity.x = imu_data["ang_vel"][0]
        msg.angular_velocity.y = imu_data["ang_vel"][1]
        msg.angular_velocity.z = imu_data["ang_vel"][2]
        msg.orientation.w = imu_data["orientation"][0]
        msg.orientation.x = imu_data["orientation"][1]
        msg.orientation.y = imu_data["orientation"][2]
        msg.orientation.z = imu_data["orientation"][3]
        self.ros_pubs["imu"].publish(msg)

    def _publish_tf(self):
        tf_msg = TFMessage()
        # odom -> base_link
        t1 = TransformStamped()
        t1.header.stamp = rospy.Time.now()
        t1.header.frame_id = "odom"
        t1.child_frame_id = "base_link"
        t1.transform.translation.x = self.odom_x
        t1.transform.translation.y = self.odom_y
        q = R.from_euler("z", self.odom_yaw).as_quat()
        t1.transform.rotation.x = q[0]
        t1.transform.rotation.y = q[1]
        t1.transform.rotation.z = q[2]
        t1.transform.rotation.w = q[3]
        tf_msg.transforms.append(t1)
        
        # base_link -> lidar
        tf_msg.transforms.append(
            self._create_tf(
                "base_link",
                "lidar",
                [0.0, 0.0, 0.4],
                [0.0, 0.0, 0.0, 1.0],
            )
        )
        
        # base_link -> imu_link
        tf_msg.transforms.append(
            self._create_tf(
                "base_link",
                "imu_link",
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            )
        )
        
        # base_link -> front_camera
        tf_msg.transforms.append(
            self._create_tf(
                "base_link",
                "front_camera",
                [0.0, 0.0, 0.5],
                [0.0, 0.0, 0.0, 1.0],
            )
        )
        
        # Link6 -> wrist_camera
        tf_msg.transforms.append(
            self._create_tf(
                "Link6",
                "wrist_camera",
                [0.0, 0.0, 0.03],
                [0.0, 0.0, 0.0, 1.0],
            )
        )
        
        self.ros_pubs["tf"].publish(tf_msg)


    def compute_wrist_cam_in_base(self):
        base_pos_w, base_quat_w = self.go2.robot.get_world_pose()
        R_base = R.from_quat(base_quat_w)

        cam_pos_w, cam_quat_w = self.wrist_camera.get_world_pose()
        R_cam = R.from_quat(cam_quat_w)

        p_rel = R_base.inv().apply(np.array(cam_pos_w) - np.array(base_pos_w))

        q_rel = (R_base.inv() * R_cam).as_quat()

        rospy.loginfo(f"[TF] wrist_camera in base_link: pos={p_rel.tolist()}, quat(xyzw)={q_rel.tolist()}")
        return p_rel, q_rel



    def _create_tf(self, frame_id, child_id, trans, rot):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = child_id
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
        t.transform.rotation.x = rot[0]
        t.transform.rotation.y = rot[1]
        t.transform.rotation.z = rot[2]
        t.transform.rotation.w = rot[3]
        return t

    def shutdown(self):
        if self.ros_initialized:
            for pub in self.ros_pubs.values():
                if pub:
                    pub.unregister()
        kit.close()

def main():
    env = SimEnvironment()
    env.load()
    env.init_ros()
    p_cam_b, q_cam_b = env.compute_wrist_cam_in_base()
    rospy.loginfo(f"Computed transform: pos={p_cam_b}, quat={q_cam_b}")
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