#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SoloFoot 无 ROS 低时延几何里程计 + 点云到地面系转换

用途
    - 直接使用底层 SDK 的 subscribeRobotState 读取关节角(+IMU)，避开 ROS 栈
    - 内置 SF_TRON1A（SoloFoot）关节几何的快速 FK，基于“支撑足接触约束 + IMU 姿态”反解基底位姿
    - 从 Realsense (pyrealsense2) 读取深度并生成点云，整体变换到地面系（odom）

适配点
    - SDK：已直接 import limxsdk（官方风格）；如需自定义导入路径，请在运行环境中正确安装/配置 limxsdk
  - 相机适配：默认用 pyrealsense2；若你有其它相机/点云源，替换 RealsenseGrabber
  - 外参与链接名：按你的装配与标定，修改 BASE_TO_CAMERA_* 与链参数

核心公式
  p^W_base = p^W_contact − R^W_b · p^b_contact
  其中 R^W_b 由 IMU 四元数直接给出；p^b_contact 由 FK(base->ankle)得到；p^W_contact 为支撑足世界锚点

注意
  - 本方案不做滤波，不做里程积分，姿态完全来自 IMU，位置由接触约束反解。适合高度场与台阶识别的“旋正到地面系”需求。
  - 支撑足切换策略这里只做了最简单的按高度判断示例，工程中建议使用接触传感/控制时序/速度阈值。
"""

from __future__ import annotations
import os
import sys
import time
import math
import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

# 可选：ROS 点云（如果你想从 /camera0/depth/color/points 读取）
try:
    import rospy
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs.point_cloud2 as pc2
    from std_msgs.msg import Header
    ROS_AVAILABLE = True
except Exception:
    ROS_AVAILABLE = False

# 可选：TF 静态变换，用于让 RViz 识别 Fixed Frame
try:
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    TF2_AVAILABLE = True
except Exception:
    TF2_AVAILABLE = False

# 直接接入 limxsdk（遵循 tron1-rl-deploy-python 的用法风格）
try:
    import limxsdk.robot.Robot as Robot
    import limxsdk.robot.RobotType as RobotType
    import limxsdk.datatypes as dt
except Exception as e:
    # 有些环境 datatypes 也可能在 limxsdk 下
    try:
        import limxsdk.robot.Robot as Robot
        import limxsdk.robot.RobotType as RobotType
        from limxsdk import datatypes as dt
    except Exception as e2:
        raise ImportError(
            "未找到 limxsdk，请先安装并确保本环境可导入（pip/whl）。原始错误: {} / {}".format(e, e2)
        )

# 可选：Realsense 相机
try:
    import pyrealsense2 as rs
    RS_AVAILABLE = True
except Exception:
    RS_AVAILABLE = False



# ============================ 配置区域 ============================

# URDF 中的链参数（取自 SF_TRON1A/urdf/robot_rl.urdf）
# 左腿各关节的 origin 偏移（父坐标系）
L_O0 = np.array([0.05556,  0.10500, -0.26020])   # abad_L_Joint origin (base)
L_O1 = np.array([-0.07700, 0.02050,  0.00000])   # hip_L_Joint origin  (abad)
L_O2 = np.array([-0.15000,-0.02050, -0.25981])   # knee_L_Joint origin (hip)
L_O3 = np.array([ 0.15000, 0.00000, -0.25981])   # ankle_L_Joint origin(knee)

# 右腿各关节的 origin 偏移
R_O0 = np.array([0.05556, -0.10500, -0.26020])   # abad_R_Joint origin (base)
R_O1 = np.array([-0.07700,-0.02050,  0.00000])   # hip_R_Joint origin  (abad)
R_O2 = np.array([-0.15000, 0.02050, -0.25981])   # knee_R_Joint origin (hip)
R_O3 = np.array([ 0.15000, 0.00000, -0.25981])   # ankle_R_Joint origin(knee)

# 关节轴方向（绕轴右手定则）
# abad: x ；hip/knee/ankle: y（注意部分关节是 -y）
AXIS_X = np.array([1.0, 0.0, 0.0])
AXIS_Y = np.array([0.0, 1.0, 0.0])
AXIS_nY = np.array([0.0,-1.0, 0.0])

# 左腿各关节轴
L_AXES = [AXIS_X, AXIS_Y, AXIS_nY, AXIS_nY]
# 右腿各关节轴（hip_R 为 -y，knee_R 为 +y，ankle_R 为 -y）
R_AXES = [AXIS_X, AXIS_nY, AXIS_Y, AXIS_nY]

# 支撑足默认选择（True=左足，False=右足），可在运行时切换
DEFAULT_LEFT_STANCE = True

# IMU 姿态来源：True=从 SDK RobotState 获得；False=单位阵（仅测试用）
IMU_FROM_SDK = True

# base -> camera 外参（替换为你的标定）
BASE_TO_CAMERA_XYZ = np.array([0.12, 0.0, 0.08])
BASE_TO_CAMERA_QUAT = np.array([0.0, 0.0, 0.0, 1.0])  # [qx,qy,qz,qw]

# 点云输出（可选保存 PLY，默认只计数）
SAVE_PLY = False
PLY_PATH = os.path.join(os.path.dirname(__file__), "points_in_odom.ply")

# 是否通过 ROS 订阅点云（True 优先使用 ROS 的 PointCloud2；False 使用 pyrealsense2 直读）
USE_ROS_POINTCLOUD = True
# ROS 点云话题名（例如 RealSense realsense-ros 默认的对齐彩色点云）
ROS_POINTCLOUD_TOPIC = "/camera0/depth/color/points"

# 强制仅走 ROS 点云，不与设备直连（避免与 realsense-ros 抢占硬件）
FORCE_ROS_ONLY = True

# ROS 点云采样（降低每帧处理量，避免回调过慢）
# 可用环境变量覆盖：ROS_CLOUD_STRIDE, ROS_CLOUD_MAX_POINTS
ROS_CLOUD_STRIDE = int(os.environ.get("ROS_CLOUD_STRIDE", "2"))            # 每隔多少个点取一个（>=1）
ROS_CLOUD_MAX_POINTS = int(os.environ.get("ROS_CLOUD_MAX_POINTS", "150000"))   # 每帧最多点数上限（防止超大帧）

# 发布频率上限（Hz），避免占用过多 CPU；可用环境变量覆盖 PUBLISH_MAX_HZ
PUBLISH_MAX_HZ = float(os.environ.get("PUBLISH_MAX_HZ", "10"))

# 是否发布变换后的点云到 ROS，便于在 RViz 中查看（Fixed Frame 设为 ROS_CLOUD_FRAME）
PUBLISH_ROS_CLOUD = True
ROS_CLOUD_TOPIC = "/world_base/points"
ROS_CLOUD_FRAME = "odom"  # RViz 的 Fixed Frame 建议设为同名（如 odom/map/world）

# 如无 TF 树提供 ROS_CLOUD_FRAME，可选地发布一个静态 TF 让 RViz 识别该帧存在
# 将 STATIC_PARENT_FRAME -> ROS_CLOUD_FRAME 发布为单位变换。
ENSURE_TF_STATIC = True
STATIC_PARENT_FRAME = os.environ.get("STATIC_PARENT_FRAME", "world")

# SDK 连接参数 真机改成LIMX_ROBOT_IP=10.192.1.2
ROBOT_IP = os.environ.get("LIMX_ROBOT_IP", "127.0.0.1")  # 真实机常见为 10.192.1.2


# ============================ 工具函数 ============================

def rot_matrix_from_axis_angle(axis: np.ndarray, theta: float) -> np.ndarray:
    """Rodrigues 公式: 轴向单位向量 axis, 旋转角 theta(rad) -> 3x3 R旋转矩阵"""
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    x, y, z = axis
    c = math.cos(theta)
    s = math.sin(theta)
    C = 1 - c
    Rm = np.array([
        [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s, c + z*z*C  ]
    ], dtype=np.float64)
    return Rm


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """四元数 [qx,qy,qz,qw] -> 3x3 R旋转矩阵"""
    qx, qy, qz, qw = q
    # 归一化
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) + 1e-12
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    Rm = np.array([
        [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
        [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
        [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
    ], dtype=np.float64)
    return Rm


def make_T(Rm: np.ndarray, t: np.ndarray) -> np.ndarray:
    """由 3x3 旋转矩阵 Rm 和 3x1 平移向量 t 构造 4x4 齐次矩阵 T"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rm
    T[:3, 3] = t
    return T


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """将 Nx3 点集用 4x4 齐次矩阵 T 变换"""
    N = pts.shape[0]
    homo = np.hstack([pts, np.ones((N,1), dtype=pts.dtype)])
    out = (T @ homo.T).T
    return out[:, :3]


# ============================ Forward Kinematics (FK) ============================

def fk_base_to_ankle_left(q_abad: float, q_hip: float, q_knee: float, q_ankle: float) -> Tuple[np.ndarray, np.ndarray]:
    """base -> ankle_L_Link 的 FK
    返回: (p_b_ankle[3], R_b_ankle[3x3])
    """
    # T0 = Trans(O0) * Rot(axis_x, q_abad)
    R0 = rot_matrix_from_axis_angle(L_AXES[0], q_abad)
    T0 = make_T(R0, L_O0)
    # T1 = Trans(O1) * Rot(+y, q_hip)
    R1 = rot_matrix_from_axis_angle(L_AXES[1], q_hip)
    T1 = make_T(R1, L_O1)
    # T2 = Trans(O2) * Rot(-y, q_knee)
    R2 = rot_matrix_from_axis_angle(L_AXES[2], q_knee)
    T2 = make_T(R2, L_O2)
    # T3 = Trans(O3) * Rot(-y, q_ankle)
    R3 = rot_matrix_from_axis_angle(L_AXES[3], q_ankle)
    T3 = make_T(R3, L_O3)

    T = (((T0 @ T1) @ T2) @ T3)
    p = T[:3, 3].copy()
    Rm = T[:3, :3].copy()
    return p, Rm


def fk_base_to_ankle_right(q_abad: float, q_hip: float, q_knee: float, q_ankle: float) -> Tuple[np.ndarray, np.ndarray]:
    """base -> ankle_R_Link 的 FK（右腿轴向与左腿略有符号差异）"""
    R0 = rot_matrix_from_axis_angle(R_AXES[0], q_abad)
    T0 = make_T(R0, R_O0)
    R1 = rot_matrix_from_axis_angle(R_AXES[1], q_hip)
    T1 = make_T(R1, R_O1)
    R2 = rot_matrix_from_axis_angle(R_AXES[2], q_knee)
    T2 = make_T(R2, R_O2)
    R3 = rot_matrix_from_axis_angle(R_AXES[3], q_ankle)
    T3 = make_T(R3, R_O3)

    T = (((T0 @ T1) @ T2) @ T3)
    p = T[:3, 3].copy()
    Rm = T[:3, :3].copy()
    return p, Rm


# ============================== SDK 接入（limxsdk） =================================

@dataclass
class RobotState:
    # 关节角（弧度）
    q_abad_L: float = 0.0
    q_hip_L: float = 0.0
    q_knee_L: float = 0.0
    q_ankle_L: float = 0.0
    q_abad_R: float = 0.0
    q_hip_R: float = 0.0
    q_knee_R: float = 0.0
    q_ankle_R: float = 0.0
    # IMU 四元数（世界->base），顺序 [qx,qy,qz,qw]
    imu_q: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


class RobotSDKWrapper:
    """基于 limxsdk 的实时订阅封装（官方主节点风格）。
    使用方法：
      sdk = RobotSDKWrapper()
      sdk.start(on_state)
      ...
        行为：
            - 官方导入风格：`import limxsdk.robot.Robot as Robot`，`import limxsdk.robot.RobotType as RobotType`。
            - 根据环境变量 ROBOT_TYPE 选择 SDK RobotType：
                    以 "SF" 开头 -> SoloFoot（若无该枚举则回退 BipedFoot）
                    以 "PF" 开头 -> PointFoot（无踝）
                    以 "WF" 开头 -> WheelFoot（若 SDK 提供）
                未设置时默认 SoloFoot（若无则回退 BipedFoot，再回退 PointFoot）。
      - 订阅 ImuData 与 RobotState，并回调上层 on_state_cb。
    """
    def __init__(self):
        self._on_state_cb = None
        self._imu_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self._robot = None

    def start(self, on_state_cb):
        self._on_state_cb = on_state_cb

        # 根据 ROBOT_TYPE 选择优先级；同时提供健壮回退到 PointFoot
        env_rt = os.environ.get("ROBOT_TYPE", "").upper().strip()
        if env_rt.startswith("SF"):
            pref_names = ('SoloFoot', 'PointFoot', 'BipedFoot')
        elif env_rt.startswith("WF"):
            pref_names = ('WheelFoot', 'PointFoot', 'SoloFoot', 'BipedFoot')
        elif env_rt.startswith("PF"):
            pref_names = ('PointFoot', 'SoloFoot', 'BipedFoot')
        else:
            pref_names = ('PointFoot', 'SoloFoot', 'BipedFoot')

        candidates = [getattr(RobotType, n, None) for n in pref_names]
        candidates = [c for c in candidates if c is not None]
        if not candidates:
            raise RuntimeError("RobotType 列表为空，请检查 limxsdk 安装是否完整")

        last_err = None
        self._robot = None
        for t in candidates:
            try:
                rob = Robot(t)
                self._robot = rob
                break
            except Exception as e:
                last_err = e
                self._robot = None
                continue
        if self._robot is None:
            raise RuntimeError(f"无法创建 Robot 实例，错误: {last_err}")

        # 获取机器人IP
        ip = os.environ.get("LIMX_ROBOT_IP", ROBOT_IP)
        if len(sys.argv) > 1:
            ip = sys.argv[1]
        # 初始化连接（个别版本若抛出 AttributeError: 'Robot' has no attribute 'robot'，
        # 说明底层对象未创建成功，尝试再次通过工厂方式创建一次）
        ok = False
        init_errors = []
        # 先用当前实例尝试
        try:
            ok = bool(self._robot.init(ip))
        except Exception as e:
            init_errors.append(str(e))
            ok = False
        # 若失败，尝试其它候选类型
        if not ok and len(candidates) > 1:
            for t in candidates[1:]:
                try:
                    rob = Robot(t)
                    if rob.init(ip):
                        self._robot = rob
                        ok = True
                        break
                except Exception as e:
                    init_errors.append(str(e))
                    continue
        if not ok:
            raise RuntimeError(f"limxsdk 连接失败，IP={ip}，尝试类型={pref_names}，错误={init_errors}")

        # IMU 回调（提供 [x,y,z,w]）
        def _imu_cb(imu: dt.ImuData):
            # 确保为 numpy 数组
            try:
                q = np.array(imu.quat, dtype=np.float64)
                if q.shape[0] == 4:
                    self._imu_quat = q
            except Exception:
                pass

        # 关节状态回调（映射到本模块 RobotState）
        def _state_cb(rs: dt.RobotState):
            try:
                q = list(rs.q)
            except Exception:
                q = []
            n = len(q)
            if n >= 8:
                # SoloFoot/BipedFoot: [L0,L1,L2,L3, R0,R1,R2,R3]
                qL0, qL1, qL2, qL3 = q[0], q[1], q[2], q[3]
                qR0, qR1, qR2, qR3 = q[4], q[5], q[6], q[7]
            elif n >= 6:
                # PointFoot: [L0,L1,L2, R0,R1,R2]，无踝，踝角置 0
                qL0, qL1, qL2, qL3 = q[0], q[1], q[2], 0.0
                qR0, qR1, qR2, qR3 = q[3], q[4], q[5], 0.0
            else:
                # 数据不足，跳过
                return

            st = RobotState(
                q_abad_L=float(qL0), q_hip_L=float(qL1), q_knee_L=float(qL2), q_ankle_L=float(qL3),
                q_abad_R=float(qR0), q_hip_R=float(qR1), q_knee_R=float(qR2), q_ankle_R=float(qR3),
                imu_q=self._imu_quat.copy()
            )
            if self._on_state_cb:
                self._on_state_cb(st)

        # 订阅
        self._robot.subscribeImuData(_imu_cb)
        self._robot.subscribeRobotState(_state_cb)

    def stop(self):
        # 当前 SDK Python 绑定未提供统一关闭接口，保留占位
        self._robot = None


# ============================ 相机（Realsense） ============================

class RealsenseGrabber:
    def __init__(self, use_color: bool=False):
        if not RS_AVAILABLE:
            raise RuntimeError("pyrealsense2 未安装，无法抓取点云。请安装 librealsense2-python 或自行替换相机接口。")
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        if use_color:
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipe.start(cfg)
        self.pc = rs.pointcloud()
        self.align = rs.align(rs.stream.depth)

    def get_points_xyz(self) -> Tuple[np.ndarray, float]:
        frames = self.pipe.wait_for_frames()
        frames = self.align.process(frames)
        depth = frames.get_depth_frame()
        if not depth:
            return np.zeros((0,3), dtype=np.float32), time.time()
        pts = self.pc.calculate(depth)
        v = np.asanyarray(pts.get_vertices())  # Nx1 的结构数组，包含 (x,y,z)
        xyz = np.asarray([(p[0], p[1], p[2]) for p in v], dtype=np.float32)
        ts = time.time()
        return xyz, ts

    def stop(self):
        self.pipe.stop()


# ============================ 相机（ROS PointCloud2） ============================

class RosPointCloudGrabber:
    """通过 ROS 订阅 PointCloud2 点云（不走 pyrealsense2），默认订阅 /camera0/depth/color/points。
    注意：需要在运行环境中可用 rospy 与 sensor_msgs。
    """
    def __init__(self, topic: str = "/camera0/depth/color/points"):
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS 未安装或不可用，无法订阅 PointCloud2。请安装 rospy/sensor_msgs 或关闭 USE_ROS_POINTCLOUD。")
        # 要求外部先初始化 ROS 节点，避免匿名节点残留难以清理
        if not rospy.core.is_initialized():
            raise RuntimeError("ROS 节点未初始化，请在创建订阅前先调用 rospy.init_node('world_base')。")
        self._topic = topic
        self._latest_xyz: Optional[np.ndarray] = None
        self._latest_ts: float = 0.0
        self._lock = threading.Lock()
        self._sub = rospy.Subscriber(self._topic, PointCloud2, self._cb, queue_size=1)

    def _cb(self, msg: PointCloud2):
        try:
            # 读取 x,y,z 字段（跳过 NaN），并进行轻量采样
            pts_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            stride = max(1, int(ROS_CLOUD_STRIDE))
            max_pts = max(1, int(ROS_CLOUD_MAX_POINTS))
            buf = []
            for i, p in enumerate(pts_iter):
                if (i % stride) != 0:
                    continue
                buf.append((float(p[0]), float(p[1]), float(p[2])))
                if len(buf) >= max_pts:
                    break
            xyz = np.asarray(buf, dtype=np.float32)
            with self._lock:
                # 存入数据
                self._latest_xyz = xyz
                # 存入时间戳，转为浮点秒
                self._latest_ts = msg.header.stamp.to_sec() if hasattr(msg.header.stamp, 'to_sec') else time.time()
        except Exception:
            pass

    def get_points_xyz(self) -> Tuple[np.ndarray, float]:
        with self._lock:
            if self._latest_xyz is None:
                return np.zeros((0, 3), dtype=np.float32), time.time()
            return self._latest_xyz.copy(), float(self._latest_ts)

    def stop(self):
        try:
            if self._sub is not None:
                self._sub.unregister()
        except Exception:
            pass


# ============================ 基底位姿估计（接触约束） ============================

class WorldEstimator:
    def __init__(self):
        self.left_stance = DEFAULT_LEFT_STANCE
        self.pW_contact_L: Optional[np.ndarray] = None
        self.pW_contact_R: Optional[np.ndarray] = None
        self.RW_b = np.eye(3, dtype=np.float64)
        self.pW_base = np.zeros(3, dtype=np.float64)
        self.TW_b = make_T(self.RW_b, self.pW_base)

    def update_imu(self, imu_q: np.ndarray):
        self.RW_b = quat_to_rotmat(imu_q.astype(np.float64))

    def compute(self, state: RobotState) -> np.ndarray:
        # 1) FK：得到 base 下的足端位置
        pL_b, _ = fk_base_to_ankle_left(state.q_abad_L, state.q_hip_L, state.q_knee_L, state.q_ankle_L)
        pR_b, _ = fk_base_to_ankle_right(state.q_abad_R, state.q_hip_R, state.q_knee_R, state.q_ankle_R)

        # 2) 初始化世界系接触锚点（上电静止时，可视为 pW_contact = R^W_b * p^b_contact）
        if self.pW_contact_L is None:
            self.pW_contact_L = self.RW_b @ pL_b
        if self.pW_contact_R is None:
            self.pW_contact_R = self.RW_b @ pR_b

        # 3) 选择支撑足（示例：谁的世界 z 更低谁是支撑足；实际建议用接触传感/控制时序）
        zL = (self.RW_b @ pL_b)[2]
        zR = (self.RW_b @ pR_b)[2]
        self.left_stance = (zL < zR)  # 左更低则选左

        if self.left_stance:
            pW_contact = self.pW_contact_L
            p_contact_b = pL_b
        else:
            pW_contact = self.pW_contact_R
            p_contact_b = pR_b

        # 4) 反解基底位置
        self.pW_base = pW_contact - (self.RW_b @ p_contact_b)
        self.TW_b = make_T(self.RW_b, self.pW_base)
        return self.TW_b

    def switch_stance_anchor(self, use_left: bool, state: RobotState):
        """支撑切换时，重置新的世界锚点以保证位姿连续"""
        self.left_stance = bool(use_left)
        if self.left_stance:
            pL_b, _ = fk_base_to_ankle_left(state.q_abad_L, state.q_hip_L, state.q_knee_L, state.q_ankle_L)
            self.pW_contact_L = self.pW_base + (self.RW_b @ pL_b)
        else:
            pR_b, _ = fk_base_to_ankle_right(state.q_abad_R, state.q_hip_R, state.q_knee_R, state.q_ankle_R)
            self.pW_contact_R = self.pW_base + (self.RW_b @ pR_b)


# ============================ 主流程 ============================

def save_ply_xyz(path: str, pts: np.ndarray):
    with open(path, 'wb') as f:
        header = """ply
format ascii 1.0
element vertex %d
property float x
property float y
property float z
end_header
""" % pts.shape[0]
        f.write(header.encode('utf-8'))
        for p in pts:
            f.write(f"{p[0]} {p[1]} {p[2]}\n".encode('utf-8'))


def main():
    print("[SoloFoot] 无ROS 低时延几何里程计 + 点云变换 启动")

    # base->camera 外参
    Rb_c = quat_to_rotmat(BASE_TO_CAMERA_QUAT.astype(np.float64))
    Tb_c = make_T(Rb_c, BASE_TO_CAMERA_XYZ.astype(np.float64))

    # SDK 订阅
    latest_state_lock = threading.Lock()
    latest_state: Optional[RobotState] = None

    def on_state_cb(st: RobotState):
        nonlocal latest_state
        with latest_state_lock:
            latest_state = st

    sdk = RobotSDKWrapper()
    sdk.start(on_state_cb)

    # 估计器
    est = WorldEstimator()

    # 如启用 ROS 点云或需要发布到 ROS，先初始化唯一的 ROS 节点（固定名，便于管理/清理）
    if ROS_AVAILABLE and (USE_ROS_POINTCLOUD or PUBLISH_ROS_CLOUD):
        if not rospy.core.is_initialized():
            try:
                rospy.init_node("world_base", anonymous=False, disable_signals=True)
            except Exception:
                pass

    # 相机选择：优先 ROS 点云（若启用且可用）-> 其次 Realsense 直读（若未强制 ROS ONLY）
    cam = None
    if USE_ROS_POINTCLOUD and ROS_AVAILABLE:
        try:
            cam = RosPointCloudGrabber(ROS_POINTCLOUD_TOPIC)
            print(f"[SoloFoot] ROS 点云订阅中: {ROS_POINTCLOUD_TOPIC}")
        except Exception as e:
            print(f"[SoloFoot] ROS 点云订阅失败: {e}")
            cam = None
    if cam is None and (not FORCE_ROS_ONLY):
        if RS_AVAILABLE:
            try:
                cam = RealsenseGrabber(use_color=False)
                print("[SoloFoot] Realsense 已启动")
            except Exception as e:
                print(f"[SoloFoot] Realsense 启动失败: {e}")
                cam = None
        else:
            print("[SoloFoot] 无可用点云源（ROS/pyrealsense2），将仅输出位姿")

    # 若需要在 RViz 中查看变换后的点云，则初始化 ROS 发布器
    ros_cloud_pub = None
    static_broadcaster = None
    if PUBLISH_ROS_CLOUD and ROS_AVAILABLE:
        try:
            # 节点已在上方统一初始化
            ros_cloud_pub = rospy.Publisher(ROS_CLOUD_TOPIC, PointCloud2, queue_size=1)
            print(f"[SoloFoot] ROS 点云发布: topic={ROS_CLOUD_TOPIC}, frame={ROS_CLOUD_FRAME}")
            # 可选：发布一个静态 TF，确保 RViz 中存在该 Fixed Frame（例如 odom）
            if ENSURE_TF_STATIC and 'ROS_CLOUD_FRAME' in globals() and TF2_AVAILABLE:
                try:
                    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
                    t = TransformStamped()
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = str(STATIC_PARENT_FRAME)
                    t.child_frame_id = str(ROS_CLOUD_FRAME)
                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    t.transform.translation.z = 0.0
                    t.transform.rotation.x = 0.0
                    t.transform.rotation.y = 0.0
                    t.transform.rotation.z = 0.0
                    t.transform.rotation.w = 1.0
                    static_broadcaster.sendTransform(t)
                    print(f"[SoloFoot] 静态 TF 发布: {STATIC_PARENT_FRAME} -> {ROS_CLOUD_FRAME} (identity)")
                except Exception:
                    pass
        except Exception as e:
            print(f"[SoloFoot] ROS 点云发布器创建失败: {e}")
            ros_cloud_pub = None

    try:
        t_last_print = 0.0
        last_cloud_ts_published = -1.0
        last_pub_wall = 0.0
        while True:
            # 读取最新状态
            with latest_state_lock:
                st = latest_state

            if st is not None:
                # IMU -> 姿态
                if IMU_FROM_SDK:
                    est.update_imu(st.imu_q)
                else:
                    est.update_imu(np.array([0,0,0,1], dtype=np.float64))
                # 估计世界->基底
                T_w_b = est.compute(st)
            else:
                # 尚未收到机器人状态，使用上一次（初始为单位）位姿
                T_w_b = est.TW_b

            # 点云处理
            if cam is not None:
                xyz_cam, ts = cam.get_points_xyz()
                # 仅在收到新帧，且不超过发布频率上限时处理
                wall_now = time.time()
                allow_rate = (wall_now - last_pub_wall) >= (1.0 / max(1e-3, PUBLISH_MAX_HZ))
                is_new = (ts > last_cloud_ts_published + 1e-6)
                if xyz_cam.size > 0 and allow_rate and is_new:
                    # 世界->相机
                    T_w_c = T_w_b @ Tb_c
                    xyz_world = transform_points(T_w_c, xyz_cam)
                    # 发布到 ROS，供 RViz 查看（Fixed Frame 设为 ROS_CLOUD_FRAME）
                    if ros_cloud_pub is not None:
                        try:
                            header = Header()
                            # 优先使用 ROS 时间
                            header.stamp = rospy.Time.now() if ROS_AVAILABLE else None
                            header.frame_id = ROS_CLOUD_FRAME
                            # 转成 ROS 点云消息（为性能可在上游加入采样/体素）
                            if ros_cloud_pub.get_num_connections() > 0:
                                cloud_msg = pc2.create_cloud_xyz32(header, xyz_world.tolist())
                                ros_cloud_pub.publish(cloud_msg)
                                last_cloud_ts_published = ts
                                last_pub_wall = wall_now
                        except Exception:
                            pass

                    if SAVE_PLY:
                        save_ply_xyz(PLY_PATH, xyz_world)

            # 打印姿态与位置（限频，仅当收到机器人状态后）
            now = time.time()
            if st is not None and (now - t_last_print > 0.2):
                t_last_print = now
                p = est.pW_base
                # 从 R 取 yaw/pitch/roll （Z-Y-X）
                Rm = est.RW_b
                yaw = math.atan2(Rm[1,0], Rm[0,0])
                pitch = -math.asin(max(-1.0, min(1.0, Rm[2,0])))
                roll = math.atan2(Rm[2,1], Rm[2,2])
                print("[Pose] p=({:.3f},{:.3f},{:.3f}) rpy(deg)=({:.1f},{:.1f},{:.1f}) stance={}".format(
                    p[0], p[1], p[2], math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                    "L" if est.left_stance else "R"))

            # 小睡以让出 CPU（整体延迟由 SDK/相机驱动决定）
            time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        sdk.stop()
        if cam is not None:
            cam.stop()
        # 释放 ROS 发布与节点
        try:
            if 'ros_cloud_pub' in locals() and ros_cloud_pub is not None:
                ros_cloud_pub.unregister()
        except Exception:
            pass
        try:
            if ROS_AVAILABLE:
                rospy.signal_shutdown('world_base_done')
                # 等待 Master 清理节点注册
                time.sleep(0.2)
        except Exception:
            pass


if __name__ == "__main__":
    main()
