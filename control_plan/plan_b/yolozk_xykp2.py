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
     → 利用机器人返回的距离进行 KP 比例前进（forward = clip(Kp * robot_z)），同时
        左右调整采用机器人返回的 x 坐标（left = -kp_x * robot_x），但当 |robot_x| 小于阈值时不进行调整，
        速度限制在 [v_min, v_max] 内。

 3️⃣ 情况 C：只要电磁铁返回的距离 magnet_z 在 (0, 1.0) 米内：
     → 利用电磁铁返回的距离进行 KP 比例前进（forward = clip(Kp * magnet_z)），同时
        左右调整采用电磁铁返回的 x 坐标（left = -kp_x * magnet_x），小于阈值时不作调整。

 4️⃣ 情况 B：检测到机器人且其距离在 (0, 0.3) 米且未检测到电磁铁（magnet_z == 0）：
     → 启动“盲推”策略，以固定前进速度 0.2 m/s 向前推进 5 秒；左右控制采用电磁铁返回的 x 坐标，
        当 |magnet_x| 小于阈值时不进行调整。

 5️⃣ 默认跟踪模式（Fallback）：
     → 若检测到目标但不满足上述条件，使用机器人返回的距离进行 KP 比例前进（裁剪后）
        同时根据目标 x/y 偏差进行左右及腿高调整（左右调整采用 turn_gain），如果 |x| 小于阈值则不调整。

 ⚙️ 控制策略说明：
 - 前进控制（除情况 B 外）均采用比例控制（v = Kp × 距离）后裁剪至 [v_min, v_max]
 - 左向控制：  
      - 情况 A 采用机器人返回的 x 坐标  
      - 情况 C 和情况 B 采用电磁铁返回的 x 坐标  
      - fallback 模式下采用目标 x 均进行调整，但当|x|低于阈值时取消修正
 - 情况 B 采用固定速度 0.2 m/s 定时盲推 5 秒，不进行实时左右调整
 - x 方向控制增加了阈值，避免小偏差噪声引起抖动

===========================================================
"""

import rclpy
from rclpy.node import Node
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

        # ===== 控制参数 =====
        self.ctrl_msg = MotionCtrl()
        self.kp = 0.5             # 前进比例控制增益，用于计算 forward = kp * distance
        self.kp_x = 0.1           # x方向横向控制增益，用于计算 left = -kp_x * target_x
        self.v_min = 0.05         # 最小前进速度（m/s）
        self.v_max = 0.5          # 最大前进速度（m/s）
        self.turn_gain = 0.05     # fallback 模式下左右修正增益
        self.leg_gain = 0.8       # fallback 模式下上下修正增益
        self.rotate_speed = 0.1   # 原地旋转速度
        self.x_threshold = 0.05   # x方向控制的阈值（m），当 |x| < 阈值时，不进行左右调整

        # ===== 目标检测数据缓存 =====
        self.current_detection = None
        # 分别存储各目标的 x、y、z 坐标，key 为 class_id（0: 电磁铁，1: 机器人）
        self.target_x_dict = {}
        self.target_y_dict = {}
        self.target_z_dict = {}

        # ===== 定时前进控制参数（用于情况B） =====
        self.forward_start_time = None   # 定时前进开始时间
        self.required_duration = 0.0       # 定时前进持续时间（秒）
        self.is_forwarding = False         # 是否处于定时前进状态

        # ===== 控制循环 10Hz =====
        self.timer = self.create_timer(0.1, self.control_loop)

    # 辅助函数：将速度裁剪在 [v_min, v_max] 范围内
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

        # 获取机器人和电磁铁的距离
        robot_z = self.target_z_dict.get(1, 0.0)
        magnet_z = self.target_z_dict.get(0, 0.0)

        # 2️⃣ 情况 A：当机器人距离 robot_z > 0.8m 时，
        # 前进速度由机器人距离计算；左右控制采用机器人返回的 x 坐标
        if robot_z > 0.8:
            forward_speed = self.clip_speed(self.kp * robot_z)
            robot_x = self.target_x_dict.get(1, 0.0)
            left_cmd = -self.kp_x * robot_x
            # 若 |robot_x| 小于阈值，则取消左右调整
            if abs(robot_x) < self.x_threshold:
                left_cmd = 0.0
            self.get_logger().info(f"[A] Robot far ({robot_z:.2f}m). Forward = {forward_speed:.2f} m/s, Left = {left_cmd:.2f} m/s (using robot x)")
            self.generate_msgs(forward_speed, left_cmd, 0.0)
            return

        # 3️⃣ 情况 C：当电磁铁返回的距离 magnet_z 在 (0, 1.0) 米内时，
        # 前进速度由电磁铁距离计算；左右控制采用电磁铁返回的 x 坐标
        if 0 < magnet_z < 1.0:
            forward_speed = self.clip_speed(self.kp * magnet_z)
            magnet_x = self.target_x_dict.get(0, 0.0)
            left_cmd = -self.kp_x * magnet_x
            if abs(magnet_x) < self.x_threshold:
                left_cmd = 0.0
            self.get_logger().info(f"[C] Magnet@{magnet_z:.2f}m → Forward = {forward_speed:.2f} m/s, Left = {left_cmd:.2f} m/s (using magnet x)")
            self.generate_msgs(forward_speed, left_cmd, 0.0)
            return

        # 4️⃣ 情况 B：当机器人距离在 (0, 0.3)m 且未检测到电磁铁（magnet_z == 0）时，
        # 采用固定前进速度 0.2 m/s，左右调整采用电磁铁返回的 x（若有），否则为 0
        if 0 < robot_z < 0.3 and magnet_z == 0:
            self.forward_speed = 0.2  # 固定前进速度
            self.required_duration = 5.0  # 定时 5 秒
            self.forward_start_time = self.get_clock().now()
            self.is_forwarding = True
            magnet_x = self.target_x_dict.get(0, 0.0)
            left_cmd = -self.kp_x * magnet_x
            if abs(magnet_x) < self.x_threshold:
                left_cmd = 0.0
            self.get_logger().info(f"[B] Too close (<0.3m) and no magnet detected. Forcing forward 5s at 0.2 m/s, Left = {left_cmd:.2f} m/s (using magnet x).")
            self.generate_msgs(self.forward_speed, left_cmd, 0.0)
            return

        # 定时前进过程（情况 B 正在进行中）
        if self.is_forwarding:
            elapsed = (self.get_clock().now() - self.forward_start_time).nanoseconds / 1e9
            if elapsed < self.required_duration:
                self.generate_msgs(self.forward_speed, 0.0, 0.0)
            else:
                self.shutdown_node()
            return

        # 5️⃣ fallback：当检测到目标但不满足上述条件时，
        # 使用机器人距离进行 KP 比例前进（裁剪后），左右和上下调整均采用 fallback 策略
        if self.current_detection:
            target_x = self.current_detection.center.x
            target_y = self.current_detection.center.y
            left_cmd = -self.turn_gain * target_x
            up_cmd = self.leg_gain * target_y
            # 同样对 fallback 左右调整进行阈值判断
            if abs(target_x) < self.x_threshold:
                left_cmd = 0.0
            forward_speed = self.clip_speed(self.kp * robot_z) if robot_z > 0 else 0.0
            self.get_logger().info(f"[TRACK] Forward = {forward_speed:.2f}, Left = {left_cmd:.2f}, Up = {up_cmd:.2f}")
            self.generate_msgs(forward_speed, left_cmd, up_cmd)
        else:
            self.generate_msgs(0.0, -self.rotate_speed, 0.0)

    # 目标检测回调：根据检测到的目标更新各数据字典
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
