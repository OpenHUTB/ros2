"""
MuJoCo 水下机器人仿真节点
========================
核心 ROS2 节点：加载 MuJoCo 物理模型，步进仿真，发布传感器数据，接收控制指令。
支持：
  1. 6-DOF 键盘遥控与浮力/洋流力学仿真
  2. 扩展传感器接口：水下多波束声呐、水下前视相机、IMU、深度计与轨迹跟踪控制
"""

import os
import math
import json
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import JointState, LaserScan, Image, Imu
from builtin_interfaces.msg import Time

try:
    import mujoco
except ImportError:
    raise ImportError(
        "\n==================================================================\n"
        "【环境依赖缺失】未检测到 MuJoCo 物理引擎模块！\n"
        "请先执行以下命令安装依赖包：\n"
        "    pip3 install -r src/water/rov_mujoco/requirements.txt\n"
        "或直接运行：\n"
        "    pip3 install mujoco numpy scipy pyyaml\n"
        "=================================================================="
    )


class MujocoSimNode(Node):
    """水下机器人 MuJoCo 仿真节点"""

    def __init__(self):
        super().__init__('mujoco_sim_node')

        # ========== 自动解析默认路径 (优先使用源码树本地 models 目录，确保最新模型与相机配置生效) ==========
        candidates = []
        this_dir = os.path.dirname(os.path.abspath(__file__))
        candidates.append(os.path.join(this_dir, '..', 'models'))
        candidates.append(os.path.join(os.getcwd(), 'models'))
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('rov_mujoco')
            candidates.append(os.path.join(share_dir, 'models'))
        except Exception:
            pass

        model_dir = None
        for c in candidates:
            c_norm = os.path.abspath(c)
            if os.path.exists(os.path.join(c_norm, 'config.json')):
                model_dir = c_norm
                break

        if model_dir is None:
            model_dir = candidates[0] if candidates else '.'

        default_model = os.path.join(model_dir, 'underwater_rov_with_arm.xml')
        default_config = os.path.join(model_dir, 'config.json')

        # ========== 参数声明 ==========
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('config_path', default_config)
        self.declare_parameter('sim_rate', 500.0)       # 物理步进频率 (Hz)
        self.declare_parameter('publish_rate', 50.0)     # 话题发布频率 (Hz)
        self.declare_parameter('use_ocean_current', True)
        self.declare_parameter('auto_trajectory_mode', False)  # 自动轨迹跟踪
        self.declare_parameter('trajectory_type', '3d_helix')  # 3d_helix / lawnmower
        self.declare_parameter('controller_type', 'nn')        # nn / pid

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        sim_rate = self.get_parameter('sim_rate').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        use_ocean_current = self.get_parameter('use_ocean_current').get_parameter_value().bool_value
        self.auto_trajectory_mode = self.get_parameter('auto_trajectory_mode').get_parameter_value().bool_value
        self.trajectory_type = self.get_parameter('trajectory_type').get_parameter_value().string_value
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        # ========== 加载配置 ==========
        if not config_path or not os.path.exists(config_path):
            self.get_logger().error(f'配置文件不存在: {config_path}')
            raise FileNotFoundError(f'配置文件不存在: {config_path}')

        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = json.load(f)

        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f'模型文件不存在: {model_path}')
            raise FileNotFoundError(f'模型文件不存在: {model_path}')

        # ========== 物理参数 ==========
        rov_cfg = self.config["rov_body"]
        self.volume = 8 * rov_cfg["half_size"][0] * rov_cfg["half_size"][1] * rov_cfg["half_size"][2]
        self.mass = rov_cfg["mass"]
        self.gravity = self.config["gravity"]

        # ========== 加载 MuJoCo 模型 ==========
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self._setup_model_params()

        # ROV body ID
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "rov")
        self.joint_id = self.model.body_jntadr[self.body_id]
        self.qpos_addr = self.model.jnt_qposadr[self.joint_id]

        # 计算整机总质量与中性浮力平衡
        self.total_mass = float(np.sum(self.model.body_mass))
        self.buoyancy_force = self.total_mass * self.gravity
        self.drag_coeff = self.config.get("ocean_drag_coeff", 10.0)

        # 实例化自动定深 PID 控制器 (悬停锁深)
        try:
            from rov_mujoco.depth_controller import PIDDepthController
        except ImportError:
            from depth_controller import PIDDepthController
        self.depth_controller = PIDDepthController(kp=1800.0, ki=35.0, kd=900.0, max_thrust=3500.0)
        self.target_depth = -1.0

        # ========== 洋流模型（可选）==========
        self.ocean_current = None
        if use_ocean_current:
            oc_cfg = self.config.get("ocean_current", {})
            if oc_cfg.get("enabled", False):
                try:
                    from rov_mujoco.ocean_current import OceanCurrentCascade, apply_current_drag
                    self.ocean_current = OceanCurrentCascade(oc_cfg)
                    self._apply_current_drag = apply_current_drag
                    self.get_logger().info('洋流模型已启用')
                except ImportError:
                    try:
                        from ocean_current import OceanCurrentCascade, apply_current_drag
                        self.ocean_current = OceanCurrentCascade(oc_cfg)
                        self._apply_current_drag = apply_current_drag
                        self.get_logger().info('洋流模型已启用 (本地导入)')
                    except ImportError as err:
                        self.get_logger().warn(f'洋流模块导入失败，已禁用: {err}')

        # ========== 扩展传感器与轨迹跟踪模型初始化 (可选) ==========
        try:
            try:
                from rov_mujoco.sonar_sensor import MultibeamSonar
                from rov_mujoco.camera_sensor import UnderwaterCamera
                from rov_mujoco.trajectory_generator import TrajectoryGenerator
                from rov_mujoco.nn_trajectory_controller import (
                    NNTrajectoryController, LOSTrajectoryController, TrackingEvaluator
                )
            except ImportError:
                from sonar_sensor import MultibeamSonar
                from camera_sensor import UnderwaterCamera
                from trajectory_generator import TrajectoryGenerator
                from nn_trajectory_controller import (
                    NNTrajectoryController, LOSTrajectoryController, TrackingEvaluator
                )

            self.sonar = MultibeamSonar(self.model, self.data, body_name='rov', num_beams=72)
            self.camera = UnderwaterCamera(self.model, self.data, camera_name='forward_cam')
            self.traj_gen = TrajectoryGenerator(trajectory_type=self.trajectory_type)

            if self.controller_type == 'pid':
                self.controller = LOSTrajectoryController()
            else:
                self.controller = NNTrajectoryController()

            self.evaluator = TrackingEvaluator(name=f"Controller-{self.controller_type.upper()}")
        except (ImportError, Exception):
            self.sonar = None
            self.camera = None
            self.traj_gen = None
            self.controller = None
            self.evaluator = None

        # ========== 初始化机械臂姿态 ==========
        self._init_robot_pose()

        # ========== 控制指令缓存 ==========
        self.cmd_vel = Twist()
        self.force_scale = 350.0
        self.torque_scale = 90.0
        self.depth_step_speed = 0.8

        # ========== 关节名称列表 ==========
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # ========== ROS2 订阅者 ==========
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        # ========== ROS2 发布者 ==========
        self.pub_odom = self.create_publisher(Odometry, '/rov/odom', 10)
        self.pub_depth = self.create_publisher(Float64, '/rov/depth', 10)
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_sonar = self.create_publisher(LaserScan, '/rov/sonar/scan', 10)
        self.pub_camera = self.create_publisher(Image, '/rov/camera/image_raw', 10)
        self.pub_imu = self.create_publisher(Imu, '/rov/imu', 10)
        self.pub_desired_path = self.create_publisher(Path, '/rov/desired_path', 10)

        # ========== 定时器 ==========
        sim_period = 1.0 / sim_rate
        publish_period = 1.0 / publish_rate
        self.sim_steps_per_publish = max(1, int(sim_rate / publish_rate))
        self.step_count = 0

        self.sim_timer = self.create_timer(sim_period, self._sim_step_callback)
        self.pub_timer = self.create_timer(publish_period, self._publish_callback)

        mode_desc = f"自主轨迹跟踪 ({self.controller_type.upper()})" if self.auto_trajectory_mode else "6-DOF 键盘遥控"
        self.get_logger().info(
            f'MuJoCo 仿真节点已启动 | 模式: {mode_desc} | 物理频率: {sim_rate}Hz | 发布频率: {publish_rate}Hz'
        )

    def _setup_model_params(self):
        """将配置文件中的参数注入 MuJoCo 模型"""
        self.model.opt.gravity = np.array([0, 0, -self.gravity])
        self.model.opt.timestep = self.config["simulation_timestep"]
        self.model.opt.density = self.config["fluid_density"]
        self.model.opt.viscosity = self.config["fluid_viscosity"]

        rov_cfg = self.config["rov_body"]
        geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, rov_cfg["geom_name"])
        self.model.geom_size[geom_id] = rov_cfg["half_size"]
        self.model.body_mass[self.model.geom_bodyid[geom_id]] = self.mass
        self.model.geom_fluid[geom_id, :5] = np.array(rov_cfg["fluid_coef"])

    def _init_robot_pose(self):
        """设定机械臂初始收纳姿态"""
        target_pose = {
            "shoulder_pan_joint": ("shoulder_pan", -0.0628),
            "shoulder_lift_joint": ("shoulder_lift", 0.126),
            "elbow_joint": ("elbow", -2.67),
            "wrist_1_joint": ("wrist_1", -0.691),
            "wrist_2_joint": ("wrist_2", -1.63),
            "wrist_3_joint": ("wrist_3", -3.27)
        }
        for jnt_name, (act_name, val) in target_pose.items():
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jnt_name)
            if jnt_id != -1:
                self.data.qpos[self.model.jnt_qposadr[jnt_id]] = val
            act_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if act_id != -1:
                self.data.ctrl[act_id] = val
        mujoco.mj_forward(self.model, self.data)

    def _cmd_vel_callback(self, msg: Twist):
        """接收速度指令"""
        self.cmd_vel = msg

    def _sim_step_callback(self):
        """物理步进回调：注入力/力矩 → mj_step"""
        dt = self.model.opt.timestep
        current_z = float(self.data.qpos[self.qpos_addr + 2])

        # 1. 中性浮力基底力注入 (完全抵消整机重力，实现自稳)
        self.data.xfrc_applied[self.body_id, :] = 0.0
        self.data.xfrc_applied[self.body_id, 2] = self.buoyancy_force

        # 2. 洋流拖曳力计算
        v_current = np.zeros(3)
        if self.ocean_current:
            pos = self.data.sensor("pos").data[:3]
            v_current = self.ocean_current.get_velocity(pos[0], pos[1], pos[2], dt)
            self._apply_current_drag(self.data, self.body_id, v_current, self.drag_coeff)

        # 3. 姿态自稳系统 (Roll/Pitch 阻尼防侧翻)
        quat = self.data.qpos[self.qpos_addr + 3: self.qpos_addr + 7]
        joint_id = self.model.body_jntadr[self.body_id]
        qvel_addr = self.model.jnt_dofadr[joint_id]
        ang_vel = self.data.qvel[qvel_addr + 3: qvel_addr + 6]
        stabilizer_torque_x = -200.0 * float(quat[1]) - 50.0 * float(ang_vel[0])
        stabilizer_torque_y = -200.0 * float(quat[2]) - 50.0 * float(ang_vel[1])
        stabilizer_torque_z = -8.0 * float(ang_vel[2])

        rot_mat = self.data.xmat[self.body_id].reshape((3, 3))

        # 4. 控制指令分流 (自主轨迹闭环跟踪 vs 6-DOF 键盘遥控)
        if self.auto_trajectory_mode:
            t = float(self.data.time)
            desired_st = self.traj_gen.get_state(t)
            pos_cur = self.data.sensor("pos").data[:3]
            vel_cur = self.data.sensor("vel").data[:3]
            yaw_cur = math.atan2(2.0*(quat[0]*quat[3] + quat[1]*quat[2]), 1.0 - 2.0*(quat[2]**2 + quat[3]**2))

            ctrl_out = self.controller.compute(
                pos_cur, vel_cur, yaw_cur,
                desired_st["pos"], desired_st["vel"], desired_st["yaw"]
            )
            self.evaluator.record(
                t, ctrl_out["err_pos"], ctrl_out["err_yaw"],
                ctrl_out["fx"], ctrl_out["fy"], ctrl_out["fz"], ctrl_out["tau_z"]
            )

            # 机体推力旋转映射至世界坐标系
            f_body = np.array([ctrl_out["fx"], ctrl_out["fy"], 0.0])
            f_world = rot_mat @ f_body

            self.data.xfrc_applied[self.body_id, 0] += f_world[0]
            self.data.xfrc_applied[self.body_id, 1] += f_world[1]
            self.data.xfrc_applied[self.body_id, 2] += ctrl_out["fz"]
            self.data.xfrc_applied[self.body_id, 3] += stabilizer_torque_x
            self.data.xfrc_applied[self.body_id, 4] += stabilizer_torque_y
            self.data.xfrc_applied[self.body_id, 5] += ctrl_out["tau_z"]
        else:
            # 6-DOF 键盘遥控与闭环定深悬停逻辑
            if abs(self.cmd_vel.linear.z) > 0.01:
                self.target_depth += self.cmd_vel.linear.z * self.depth_step_speed * dt
                thrust_z = self.cmd_vel.linear.z * self.force_scale
            else:
                thrust_z = self.depth_controller.compute(
                    self.target_depth, current_z, dt, current_velocity=float(v_current[2])
                )

            f_body = np.array([
                self.cmd_vel.linear.x * self.force_scale,
                self.cmd_vel.linear.y * self.force_scale,
                0.0
            ])
            f_world = rot_mat @ f_body

            tau_body = np.array([
                self.cmd_vel.angular.x * self.torque_scale,
                self.cmd_vel.angular.y * self.torque_scale,
                self.cmd_vel.angular.z * self.torque_scale
            ])
            tau_world = rot_mat @ tau_body

            self.data.xfrc_applied[self.body_id, 0] += f_world[0]
            self.data.xfrc_applied[self.body_id, 1] += f_world[1]
            self.data.xfrc_applied[self.body_id, 2] += thrust_z
            self.data.xfrc_applied[self.body_id, 3] += tau_world[0] + stabilizer_torque_x
            self.data.xfrc_applied[self.body_id, 4] += tau_world[1] + stabilizer_torque_y
            self.data.xfrc_applied[self.body_id, 5] += tau_world[2] + stabilizer_torque_z

        # 5. 物理积分推进
        mujoco.mj_step(self.model, self.data)
        self.step_count += 1

    def _publish_callback(self):
        """发布传感器数据 (Odom, Depth, IMU, Sonar, Camera, Path)"""
        now = self.get_clock().now().to_msg()
        pos = self.data.sensor("pos").data[:3]
        vel = self.data.sensor("vel").data[:3]
        quat = self.data.qpos[self.qpos_addr + 3: self.qpos_addr + 7]

        # --- 里程计 ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'rov_base'
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.w = float(quat[0])
        odom.pose.pose.orientation.x = float(quat[1])
        odom.pose.pose.orientation.y = float(quat[2])
        odom.pose.pose.orientation.z = float(quat[3])
        odom.twist.twist.linear.x = float(vel[0])
        odom.twist.twist.linear.y = float(vel[1])
        odom.twist.twist.linear.z = float(vel[2])
        self.pub_odom.publish(odom)

        # --- 水深 ---
        depth_msg = Float64()
        depth_msg.data = float(pos[2])
        self.pub_depth.publish(depth_msg)

        # --- 惯性测量单元 (IMU) ---
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'rov_imu_link'
        imu_msg.orientation.w = float(quat[0])
        imu_msg.orientation.x = float(quat[1])
        imu_msg.orientation.y = float(quat[2])
        imu_msg.orientation.z = float(quat[3])
        try:
            gyro = self.data.sensor("gyro").data[:3]
            accel = self.data.sensor("accel").data[:3]
            imu_msg.angular_velocity.x = float(gyro[0])
            imu_msg.angular_velocity.y = float(gyro[1])
            imu_msg.angular_velocity.z = float(gyro[2])
            imu_msg.linear_acceleration.x = float(accel[0])
            imu_msg.linear_acceleration.y = float(accel[1])
            imu_msg.linear_acceleration.z = float(accel[2])
        except Exception:
            pass
        self.pub_imu.publish(imu_msg)

        # --- 多波束前视声呐 (/rov/sonar/scan) ---
        if self.sonar is not None:
            self.sonar.update()
            header = Header()
            header.stamp = now
            header.frame_id = 'rov_sonar_link'
            scan_msg = self.sonar.create_laserscan_msg(header)
            self.pub_sonar.publish(scan_msg)

        # --- 前视摄像头 (降采样每 5 次步进发布一次) ---
        if self.camera is not None and self.step_count % 5 == 0:
            header = Header()
            header.stamp = now
            header.frame_id = 'rov_camera_optical_link'
            cam_msg = self.camera.create_image_msg(header)
            self.pub_camera.publish(cam_msg)

        # --- 参考期望航线 Path (每 50 步广播一次) ---
        if self.traj_gen is not None and self.step_count % 50 == 0:
            header = Header()
            header.stamp = now
            header.frame_id = 'world'
            path_msg = self.traj_gen.generate_path_msg(header)
            self.pub_desired_path.publish(path_msg)

        # --- 机械臂关节状态 ---
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = self.joint_names
        joint_state.position = []
        joint_state.velocity = []
        for jnt_name in self.joint_names:
            jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jnt_name)
            if jnt_id != -1:
                qadr = self.model.jnt_qposadr[jnt_id]
                vadr = self.model.jnt_dofadr[jnt_id]
                joint_state.position.append(float(self.data.qpos[qadr]))
                joint_state.velocity.append(float(self.data.qvel[vadr]))
        self.pub_joint_states.publish(joint_state)
