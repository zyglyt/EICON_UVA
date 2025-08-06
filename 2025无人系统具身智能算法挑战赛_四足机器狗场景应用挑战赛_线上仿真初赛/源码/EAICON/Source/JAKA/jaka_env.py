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
import io
import os
import carb
import torch
import numpy as np
import random
import math
import time
from typing import List, Optional, Callable, Tuple

from scipy.spatial import cKDTree
from scipy.interpolate import CubicSpline

import omni.graph.core as og
import omni.physics.tensors as physics
from pxr import UsdGeom, Gf, UsdPhysics, Sdf, PhysicsSchemaTools

from omni.physx import get_physx_scene_query_interface
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.motion_generation import RmpFlow, ArticulationMotionPolicy
from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
import omni

class _Node:
    __slots__ = ('config', 'parent', 'cost')
    def __init__(self, config: np.ndarray, parent: Optional['_Node']=None, cost: float=0.0):
        self.config = config
        self.parent = parent
        self.cost = cost

def _euclidean(a: np.ndarray, b: np.ndarray) -> float:
    return np.linalg.norm(a - b)

def _collision_free(a: np.ndarray, b: np.ndarray, validator: Callable[[np.ndarray], bool]) -> bool:
    d = np.linalg.norm(b - a)
    steps = max(2, int(d / 0.05)) 
    for t in np.linspace(0, 1, steps):
        if not validator(a * (1 - t) + b * t):
            return False
    return True

def _smooth_path(path: List[np.ndarray], validator: Callable[[np.ndarray], bool], iterations: int = 50) -> List[np.ndarray]:
    for _ in range(iterations):
        if len(path) < 3:
            break
        i = random.randint(0, len(path) - 3)
        j = random.randint(i + 2, len(path) - 1)
        if _collision_free(path[i], path[j], validator):
            path = path[:i+1] + path[j:]
    return path

def trrt_star_optimized(
    start: np.ndarray,
    goal: np.ndarray,
    validator: Callable[[np.ndarray], bool],
    joint_limits: List[Tuple[float, float]],
    max_iter: int = 5000,
    step_size: float = 0.1,
    radius: float = 0.5,
    init_temp: float = 1.0,
    k0: float = 0.1,
    alpha: float = 0.95,
    beta: float = 1.1,
) -> Optional[List[np.ndarray]]:

    tree: List[_Node] = [_Node(start.copy())]
    configs = [start.copy()]
    temperature = init_temp
    dim = len(start)

    for i in range(max_iter):
        k = k0 + (1 - k0) * (i / max_iter)
        if random.random() < k:
            sample = goal.copy()
        else:
            sample = np.array([random.uniform(joint_limits[d][0], joint_limits[d][1]) for d in range(dim)])
        kd = cKDTree(configs)
        _, idx = kd.query(sample, k=1)
        nearest_node = tree[idx]
        
        direction = sample - nearest_node.config
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            continue
        new_config = nearest_node.config + (direction / dist) * min(step_size, dist)

        if not validator(new_config):
            continue
            
        delta_cost = _euclidean(nearest_node.config, new_config)
        if delta_cost > 0 and math.exp(-delta_cost / temperature) < random.random():
            temperature *= beta 
            continue
        temperature = max(temperature * alpha, 1e-3) 
        
        new_cost = nearest_node.cost + delta_cost

        neighbor_indices = kd.query_ball_point(new_config, r=radius)
        best_parent = nearest_node
        min_cost = new_cost
        
        for neighbor_idx in neighbor_indices:
            neighbor_node = tree[neighbor_idx]
            if _collision_free(neighbor_node.config, new_config, validator):
                cost_through_neighbor = neighbor_node.cost + _euclidean(neighbor_node.config, new_config)
                if cost_through_neighbor < min_cost:
                    min_cost = cost_through_neighbor
                    best_parent = neighbor_node

        new_node = _Node(new_config.copy(), parent=best_parent, cost=min_cost)
        tree.append(new_node)
        configs.append(new_config.copy())
        
        for neighbor_idx in neighbor_indices:
            if neighbor_idx == idx: continue 
            neighbor_node = tree[neighbor_idx]
            if _collision_free(new_config, neighbor_node.config, validator):
                rewire_cost = new_node.cost + _euclidean(new_config, neighbor_node.config)
                if rewire_cost < neighbor_node.cost:
                    neighbor_node.parent = new_node
                    neighbor_node.cost = rewire_cost

        if _euclidean(new_config, goal) < step_size:
            if _collision_free(new_config, goal, validator):
                goal_node = _Node(goal.copy(), parent=new_node, cost=new_node.cost + _euclidean(new_config, goal))
                path = []
                curr = goal_node
                while curr is not None:
                    path.append(curr.config)
                    curr = curr.parent
                path.reverse()
                return _smooth_path(path, validator)

    return None

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([x, y, z, w])

def quat_inverse(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([-x, -y, -z, w])

def calculate_relative_pos(target_pos: np.ndarray, ee_pos: np.ndarray, ee_quat: np.ndarray) -> np.ndarray:
    from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices
    R_ee_w = quats_to_rot_matrices(ee_quat) 
    delta_world = target_pos - ee_pos
    return R_ee_w.T @ delta_world

def calculate_relative_quat(start_quat: np.ndarray, current_quat: np.ndarray) -> np.ndarray:
    return quat_multiply(current_quat, quat_inverse(start_quat))

def calculate_relative_pose(ee_pos: np.ndarray, ee_quat: np.ndarray, target_first_quat: np.ndarray, delta_pos_local: np.ndarray, quat_relative_from_start: np.ndarray) -> (np.ndarray, np.ndarray):
    from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices
    R_ee_w = quats_to_rot_matrices(ee_quat)
    pos_new = ee_pos + R_ee_w @ delta_pos_local
    quat_new = quat_multiply(quat_relative_from_start, target_first_quat)
    return pos_new, quat_new

class JakaRmpFlowController:
    def __init__(
        self,
        stage,
        robot_arm_prim_path: str,
        robot_arm_yaml_path: str,
        robot_arm_urdf_path: str,
        robot_arm_rmpflow_config_path: str,
        end_effector_name: str,
        gripper_joint_names: List[str],
        gripper_collisions_names: List[str],
    ):
        self.stage = stage
        self.num_arm_dof = 6
        self._gripper_open, self._gripper_closed = 0.0, 0.001

        self._articulation = Articulation(prim_path=robot_arm_prim_path)

        rmp_cfg = dict(
            end_effector_frame_name="gripper_center",
            maximum_substep_size=0.01,
            ignore_robot_state_updates=True,
            robot_description_path=robot_arm_yaml_path,
            urdf_path=robot_arm_urdf_path,
            rmpflow_config_path=robot_arm_rmpflow_config_path,
        )
        self._rmpflow  = RmpFlow(**rmp_cfg)
        self._art_rmp  = ArticulationMotionPolicy(self._articulation,
                                                  self._rmpflow,
                                                  self.num_arm_dof)
        self._kin_solver = self._rmpflow.get_kinematics_solver()
        self._art_kin    = ArticulationKinematicsSolver(
            self._articulation, self._kin_solver, end_effector_name
        )

        from isaacsim.sensors.physics.scripts.effort_sensor import EffortSensor
        self.left_sensor  = EffortSensor(
            prim_path=gripper_joint_names[0], sensor_period=1 / 30,
            use_latest_data=True, enabled=True
        )
        self.right_sensor = EffortSensor(
            prim_path=gripper_joint_names[1], sensor_period=1 / 30,
            use_latest_data=True, enabled=True
        )

        self.gripper_center_collision = gripper_collisions_names[0]
        self.gripper_is_close = False
        self.target = None
        self.overlap_prim_path = ""
        self.delta_pos_local = np.zeros(3)
        self.ee_first_quat   = np.zeros(4)
        self.target_first_quat = np.zeros(4)

        self.trajectory            = None
        self.time_from_start       = None
        self.trajectory_duration   = 0.0
        self.trajectory_start_time = None
        self._tracking_in_progress = False

        from omni.isaac.core.simulation_context import SimulationContext
        self._sim_ctx = SimulationContext.instance()

        self.dof_count = None

    def _pack_full(self, arm_q: np.ndarray, grip_val: float) -> np.ndarray:
        if self.dof_count >= 8:       
            return np.concatenate([arm_q, [grip_val, grip_val]])
        return np.concatenate([arm_q, [grip_val]])

    def get_articulation(self) -> Articulation:
        return self._articulation
    def forward_and_track(self, gripper_value: float | None = None):
        grip_cmd = (
            gripper_value                          
            if gripper_value is not None
            else (self._gripper_closed              
                if self.gripper_is_close
                else self._gripper_open)
        )
        self.update_trajectory_tracking(grip_cmd)
        tup = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(self.gripper_center_collision))
        get_physx_scene_query_interface().overlap_mesh(
            tup[0], tup[1], self.on_gripper_center_hit, False
        )

        left_eff, right_eff = self.get_gripper_efforts()

        TH = 0.2 
        if (left_eff >= TH or right_eff >= TH):
            if self.overlap_prim_path and not self.gripper_is_close:
                self.close_gripper()   
            self.gripper_is_close = True
        else:
            if self.gripper_is_close:
                self.open_gripper()
            self.gripper_is_close = False

        if self.gripper_is_close and self.target:
            ep, eq = self.get_end_effector_pose()
            rq      = calculate_relative_quat(self.ee_first_quat, eq)
            np_, nq = calculate_relative_pose(
                ep, eq, self.target_first_quat, self.delta_pos_local, rq
            )
            self.target.set_world_pose(np_, nq)


    def reset(self):
        if not getattr(self, "_initialized", False):
            self._articulation.initialize()
            self._articulation.set_joints_default_state(
                positions=np.array([
                    0.0, 1.3, -1.5, 4.5, 1.6, 0.0,
                    self._gripper_open, self._gripper_open
                ])
            )
            self.dof_count   = len(self._articulation.dof_names)
            self._initialized = True           
        self._articulation.post_reset()
        base_p, base_q = self._articulation.get_world_pose()
        if self._kin_solver:
            self._kin_solver.set_robot_base_pose(base_p, base_q)
    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        p, m = self._art_kin.compute_end_effector_pose()
        from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats
        q_xyzw = rot_matrices_to_quats(m)
        if p.ndim > 1:     
            p = p[0]
        if q_xyzw.ndim > 1: 
            q_xyzw = q_xyzw[0]
        q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
        return p, q_wxyz
   

    def _pack_full(self, arm_q: np.ndarray, grip_val: float):
        if self.dof_count >= 8:         
            return np.concatenate([arm_q, [grip_val, grip_val]])   
        else:                         
            return np.concatenate([arm_q, [grip_val]])
    def plan_and_execute_trajectory(
        self,
        target_pos: np.ndarray,
        target_rot_wxyz: np.ndarray,
        duration: float = 5.0
    ):
        if self._tracking_in_progress:
            carb.log_warn("正在执行轨迹，请等待完成后再规划新目标。")
            return

        current_joint_pos = self._articulation.get_joint_positions()
        if current_joint_pos is None:
            carb.log_error("无法获取当前关节位置，规划中止。")
            return
        start_config = current_joint_pos[:self.num_arm_dof]

        target_quat_xyzw = np.array([
            target_rot_wxyz[1], target_rot_wxyz[2],
            target_rot_wxyz[3], target_rot_wxyz[0]
        ])
        action, ok = self._art_kin.compute_inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_quat_xyzw
        )
        if not ok:
            carb.log_error("IK 解算失败，无法规划路径。")
            return
        goal_config = action.joint_positions
        try:
            limits = self._articulation.get_dof_limits()
            arm_limits = limits[: self.num_arm_dof].tolist()

        except AttributeError:
            props = (self._articulation.dof_properties
                    if hasattr(self._articulation, "dof_properties")
                    else self._articulation.get_dof_properties())
            lower = props["lower"][: self.num_arm_dof]
            upper = props["upper"][: self.num_arm_dof]
            arm_limits = list(zip(lower, upper))

        raw_path = trrt_star_optimized(
            start=start_config,
            goal=goal_config,
            validator=lambda q: True,
            joint_limits=arm_limits,
            step_size=0.01, radius=0.5
        )
        if raw_path is None:
            carb.log_error("TRRT* 未能找到有效路径。")
            return

        N     = len(raw_path)
        times = np.linspace(0, duration, N)
        traj  = np.stack(raw_path)
        cs_list = [CubicSpline(times, traj[:, d]) for d in range(self.num_arm_dof)]
        t_query = np.linspace(0, duration, max(100, int(duration * 100)))
        positions = np.vstack([cs(t_query) for cs in cs_list]).T

        self.trajectory            = positions          
        self.time_from_start       = t_query
        self.trajectory_duration   = duration
        self.trajectory_start_time = None
        self._tracking_in_progress = True
        carb.log_warn(f"路径规划成功，生成 {len(self.trajectory)} 个轨迹点，开始轨迹跟踪。")

    def update_trajectory_tracking(self, gripper_value: float):
        if not self._tracking_in_progress:
            return

        if self.trajectory_start_time is None:
            self.trajectory_start_time = self._sim_ctx.current_time
        elapsed = self._sim_ctx.current_time - self.trajectory_start_time

        if elapsed >= self.trajectory_duration:
            carb.log_warn("轨迹跟踪完成。")
            final_arm_pos = self.trajectory[-1]
            full_pos      = self._pack_full(final_arm_pos, gripper_value)
            self._articulation.get_articulation_controller().apply_action(
                ArticulationAction(joint_positions=full_pos)
            )
            self._tracking_in_progress = False
            self.trajectory = None
            return

        idx = np.searchsorted(self.time_from_start, elapsed, side="right")
        if idx == 0:
            target_arm_pos = self.trajectory[0]
        elif idx >= len(self.time_from_start):
            target_arm_pos = self.trajectory[-1]
        else:
            t0, t1   = self.time_from_start[idx-1], self.time_from_start[idx]
            alpha    = (elapsed - t0) / (t1 - t0) if (t1 - t0) > 0 else 0
            target_arm_pos = (1 - alpha) * self.trajectory[idx-1] + alpha * self.trajectory[idx]

        full_joint_positions = self._pack_full(target_arm_pos, gripper_value)
        self._articulation.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=full_joint_positions)
        )


    def on_gripper_center_hit(self, hit):
        prefix = "/World/RobotArm/JAKA_C5_With_DH_PGI/"
        if prefix in hit.rigid_body:
            pass
        else:
            self.overlap_prim_path = hit.rigid_body
        return True

    def get_gripper_efforts(self):
        left  = self.left_sensor.get_sensor_reading(use_latest_data=True)
        right = self.right_sensor.get_sensor_reading(use_latest_data=True)
        return (left.value if left.is_valid else 0.0,
                right.value if right.is_valid else 0.0)

    def open_gripper(self):
        if not self.gripper_is_close:
            return
        prim = self.stage.GetPrimAtPath(self.overlap_prim_path)
        if prim.IsValid():
            UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(True)
        self.gripper_is_close  = False
        self.target            = None
        self.overlap_prim_path = ""

    def close_gripper(self):
        if self.gripper_is_close:
            return
        if self.overlap_prim_path:
            prim = self.stage.GetPrimAtPath(self.overlap_prim_path)
            if prim.IsValid():
                UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
                from omni.isaac.core.prims import XFormPrim
                self.target = XFormPrim(prim_path=self.overlap_prim_path)
                tp, tq = self.target.get_world_pose()
                ep, eq = self.get_end_effector_pose()
                self.delta_pos_local   = calculate_relative_pos(tp, ep, eq)
                self.ee_first_quat     = eq.copy()
                self.target_first_quat = tq.copy()
        self.gripper_is_close = True