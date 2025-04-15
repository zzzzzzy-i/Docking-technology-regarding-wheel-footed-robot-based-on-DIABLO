"""
===========================================================
 🦾 Diablo 机器人自主导航控制逻辑（基于YOLO目标检测）

 📌 功能概述：
 本节点接收来自目标检测节点（YOLO识别）的目标信息，根据检测到的目标类别
（电磁铁 class_id=0，机器人 class_id=1）及其空间位置信息，自动控制机器人前进、
转向、腿高调节等，实现基于视觉的自主导航与跟踪控制。

 📍 控制流程逻辑（按优先级）：

 1️⃣ 未检测到机器人（class_id=1）和电磁铁（class_id=0）：
     → 原地顺时针旋转，持续扫描环境，尝试重新识别目标。

 2️⃣ 情况 A：检测到机器人，且其距离 robot_z > 0.8 米：
     → 利用机器人返回的距离进行比例前进（forward = clip(kp * robot_z)），同时
        左右调整采用机器人返回的 x 坐标（left = -kp_x * robot_x），并对 x 方向误差设有阈值，
        前进速度限制在 [v_min, v_max] 内。

 3️⃣ 情况 C：只要电磁铁返回的距离 magnet_z 在 (0, 1.0) 米内：
     → 利用电磁铁返回的距离进行比例前进（forward = clip(kp * magnet_z)），同时
        左右调整采用电磁铁返回的 x 坐标（left = -kp_x * magnet_x），并对 x 误差进行阈值判断。

 4️⃣ 情况 B：检测到机器人且其距离在 (0, 0.3) 米且未检测到电磁铁（magnet_z == 0）：
     → 启动“盲推”策略，以固定前进速度 0.2 m/s 向前推进 5 秒；左右调整采用电磁铁返回的 x，
        当 |x| 小于阈值时不作调整。

 5️⃣ 默认跟踪模式（Fallback）：
     → 若检测到目标但不满足上述条件，使用机器人返回的距离进行前进（比例前进，经剪裁），
        同时根据目标 x/y 偏差进行左右及腿高调整（fallback 左右调整采用 turn_gain），当 |x| 小于阈值时取消调整。

 ⚙️ 控制策略说明：
 - 除情况 B 外，前进速度均采用比例控制（v = kp × 距离），后剪裁至 [v_min, v_max]。
 - 左向控制在情况 A采用机器人返回的 x；情况 C和B采用电磁铁返回的 x；fallback 同样根据目标 x 调整。
 - 所有关键参数均通过 ROS 参数接口进行动态调节，无需重启节点。

===========================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from motion_msgs.msg import MotionCtrl
from detectionresult.msg import DetectionResult

class AutoNavNode(Node):
    def __init__(self):
        super().__init__('diablo_auto_nav_node')

        # ===== ROS 通信接口 =====
        self.publisher = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 2)
        self.subscription = self.create_subscription(
            DetectionResult,
            '/detection_result',
            self.target_callback,
            10)

        # ===== 动态参数声明 =====
        # 可以在节点启动前通过命令行参数或者参数服务器传入
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('kp_x', 0.1)
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('x_threshold', 0.05)
        self.declare_parameter('turn_gain', 0.05)
        self.declare_parameter('leg_gain', 0.8)
        self.declare_parameter('rotate_speed', 0.1)

        # 读取参数初始值
        self.kp = self.get_parameter('kp').value
        self.kp_x = self.get_parameter('kp_x').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value
        self.x_threshold = self.get_parameter('x_threshold').value
        self.turn_gain = self.get_parameter('turn_gain').value
        self.leg_gain = self.get_parameter('leg_gain').value
        self.rotate_speed = self.get_parameter('rotate_speed').value

        # 添加参数更新回调，实现动态调节
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ===== 控制参数 =====
        self.ctrl_msg = MotionCtrl()

        # ===== 目标检测数据缓存 =====
        self.current_detection = None
        self.target_x_dict = {}   # key 为 class_id；存储 x 坐标
        self.target_y_dict = {}   # 存储 y 坐标
        self.target_z_dict = {}   # 存储 z 坐标（用于距离判断）

        # ===== 定时前进控制参数（用于情况B） =====
        self.forward_start_time = None
        self.required_duration = 0.0
        self.is_forwarding = False

        # ===== 控制循环 10Hz =====
        self.timer = self.create_timer(0.1, self.control_loop)

    # 参数更新回调函数，允许动态调节关键参数
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp' and param.type_ == Parameter.Type.DOUBLE:
                self.kp = param.value
                self.get_logger().info(f"Parameter kp updated to {self.kp}")
            elif param.name == 'kp_x' and param.type_ == Parameter.Type.DOUBLE:
                self.kp_x = param.value
                self.get_logger().info(f"Parameter kp_x updated to {self.kp_x}")
            elif param.name == 'v_min' and param.type_ == Parameter.Type.DOUBLE:
                self.v_min = param.value
                self.get_logger().info(f"Parameter v_min updated to {self.v_min}")
            elif param.name == 'v_max' and param.type_ == Parameter.Type.DOUBLE:
                self.v_max = param.value
                self.get_logger().info(f"Parameter v_max updated to {self.v_max}")
            elif param.name == 'x_threshold' and param.type_ == Parameter.Type.DOUBLE:
                self.x_threshold = param.value
                self.get_logger().info(f"Parameter x_threshold updated to {self.x_threshold}")
            elif param.name == 'turn_gain' and param.type_ == Parameter.Type.DOUBLE:
                self.turn_gain = param.value
                self.get_logger().info(f"Parameter turn_gain updated to {self.turn_gain}")
            elif param.name == 'leg_gain' and param.type_ == Parameter.Type.DOUBLE:
                self.leg_gain = param.value
                self.get_logger().info(f"Parameter leg_gain updated to {self.leg_gain}")
            elif param.name == 'rotate_speed' and param.type_ == Parameter.Type.DOUBLE:
                self.rotate_speed = param.value
                self.get_logger().info(f"Parameter rotate_speed updated to {self.rotate_speed}")
        return rclpy.parameter.SetParametersResult(successful=True)

    # 辅助函数：将前进速度裁剪在 [v_min, v_max] 范围内
    def clip_speed(self, speed: float) -> float:
        return max(self.v_min, min(self.v_max, speed))

    # 发布运动控制消息
    def generate_msgs(self, forward: float, left: float, up: float):
        self.ctrl_msg.value.forward = forward
        self.ctrl_msg.value.left = left
        self.ctrl_msg.value.up = up
        self.publisher.publish(self.ctrl_msg)
        self.get_logger().debug(f"CMD: Fwd={forward:.2f}, Lft={left:.2f}, Up={up:.2f}")

    # 停止运动并安全关闭节点
    def shutdown_node(self):
        self.get_logger().info("Mission complete. Stopping and shutting down.")
        self.generate_msgs(0.0, 0.0, 0.0)
        self.destroy_node()
        rclpy.shutdown()

    # 主控制逻辑循环
    def control_loop(self):
        # 1️⃣ 未检测到任何目标（机器人与电磁铁） → 原地旋转
        if (0 not in self.target_z_dict) and (1 not in self.target_z_dict):
            self.get_logger().info("No robot or magnet detected. Rotating.")
            self.generate_msgs(0.0, -self.rotate_speed, 0.0)
            return

        # 获取机器人和电磁铁的距离（z 坐标）
        robot_z = self.target_z_dict.get(1, 0.0)
        magnet_z = self.target_z_dict.get(0, 0.0)

        # 2️⃣ 情况 A：当机器人距离 robot_z > 0.8m 时，
        # 前进速度由机器人距离计算；左右控制采用机器人返回的 x 坐标，并判断阈值
        if robot_z > 0.8:
            forward_speed = self.clip_speed(self.kp * robot_z)
            robot_x = self.target_x_dict.get(1, 0.0)
            left_cmd = -self.kp_x * robot_x
            if abs(robot_x) < self.x_threshold:
                left_cmd = 0.0
            self.get_logger().info(f"[A] Robot far ({robot_z:.2f}m). Forward = {forward_speed:.2f} m/s, Left = {left_cmd:.2f} m/s (using robot x)")
            self.generate_msgs(forward_speed, left_cmd, 0.0)
            return

        # 3️⃣ 情况 C：当电磁铁返回的距离 magnet_z 在 (0, 1.0) 米内时，
        # 前进速度由电磁铁距离计算；左右控制采用电磁铁返回的 x 坐标（带阈值处理）
        if 0 < magnet_z < 1.0:
            forward_speed = self.clip_speed(self.kp * magnet_z)
            magnet_x = self.target_x_dict.get(0, 0.0)
            left_cmd = -self.kp_x * magnet_x
            if abs(magnet_x) < self.x_threshold:
                left_cmd = 0.0
            self.get_logger().info(f"[C] Magnet@{magnet_z:.2f}m → Forward = {forward_speed:.2f} m/s, Left = {left_cmd:.2f} m/s (using magnet x)")
            self.generate_msgs(forward_speed, left_cmd, 0.0)
            return

        # 4️⃣ 情况 B：当机器人距离在 (0, 0.3)m 且未检测到电磁铁时，
        # 采用固定前进速度 0.2 m/s 定时 5 秒；左右控制采用电磁铁返回的 x 坐标（带阈值处理）
        if 0 < robot_z < 0.3 and magnet_z == 0:
            self.forward_speed = 0.2
            self.required_duration = 5.0
            self.forward_start_time = self.get_clock().now()
            self.is_forwarding = True
            magnet_x = self.target_x_dict.get(0, 0.0)
            left_cmd = -self.kp_x * magnet_x
            if abs(magnet_x) < self.x_threshold:
                left_cmd = 0.0
            self.get_logger().info(f"[B] Too close (<0.3m) and no magnet detected. Forcing forward 5s at 0.2 m/s, Left = {left_cmd:.2f} m/s (using magnet x).")
            self.generate_msgs(self.forward_speed, left_cmd, 0.0)
            return

        # 定时前进过程中（情况 B 执行的后续循环）
        if self.is_forwarding:
            elapsed = (self.get_clock().now() - self.forward_start_time).nanoseconds / 1e9
            if elapsed < self.required_duration:
                self.generate_msgs(self.forward_speed, 0.0, 0.0)
            else:
                self.shutdown_node()
            return

        # 5️⃣ fallback 模式：当检测到目标但不满足上述条件时，
        # 使用机器人返回的距离进行前进，并根据目标 x/y 进行左右和上下微调（左右调整带阈值）
        if self.current_detection:
            target_x = self.current_detection.center.x
            target_y = self.current_detection.center.y
            left_cmd = -self.turn_gain * target_x
            up_cmd = self.leg_gain * target_y
            if abs(target_x) < self.x_threshold:
                left_cmd = 0.0
            forward_speed = self.clip_speed(self.kp * robot_z) if robot_z > 0 else 0.0
            self.get_logger().info(f"[TRACK] Forward = {forward_speed:.2f}, Left = {left_cmd:.2f}, Up = {up_cmd:.2f}")
            self.generate_msgs(forward_speed, left_cmd, up_cmd)
        else:
            self.generate_msgs(0.0, -self.rotate_speed, 0.0)

    # 目标检测回调：根据传入的检测结果更新目标数据
    def target_callback(self, msg: DetectionResult):
        self.current_detection = msg
        class_id = msg.class_id
        self.target_x_dict[class_id] = msg.center.x
        self.target_y_dict[class_id] = msg.center.y
        self.target_z_dict[class_id] = msg.center.z
        self.get_logger().info(f"[DETECT] Class {class_id}: x = {msg.center.x:.2f}, y = {msg.center.y:.2f}, z = {msg.center.z:.2f}")

# ROS 入口
def main(args=None):
    rclpy.init(args=args)
    node = AutoNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
