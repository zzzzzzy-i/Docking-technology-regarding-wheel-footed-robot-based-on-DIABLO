"""
===========================================================
 🦾 Diablo 机器人自主导航控制逻辑（基于YOLO目标检测）

 📌 功能概述：
 本节点接收来自目标检测节点（YOLO识别）的目标信息，根据检测到的目标类别
（电磁铁 class_id=0，机器人 class_id=1）及其空间位置，自动控制机器人前进、
转向、腿高调节等，实现基于视觉的自主导航与跟踪控制。

 📍 控制流程逻辑（按优先级）：

 1️⃣ 未检测到机器人（class_id=1）和电磁铁（class_id=0）：
     → 原地顺时针旋转，持续扫描环境，尝试重新识别目标。

 2️⃣ 情况 A：检测到机器人，且其距离 robot_z > 0.8 米：
     → 使用 robot_z 进行 KP 比例控制（v = Kp * robot_z），速度限制在 [v_min, v_max] 内。

 3️⃣ 情况 C：机器人距离在 (0, 1.0) 米之间，且检测到电磁铁（magnet_z ≠ 0）：
     → 使用电磁铁距离 magnet_z 进行 KP 比例控制，速度裁剪。

 4️⃣ 情况 B：机器人距离在 (0, 0.3) 米之间，且未检测到电磁铁（magnet_z = 0）：
     → 启动“盲推”策略，以固定速度 0.2 m/s 向前推进 5 秒。

 5️⃣ 默认跟踪模式（Fallback）：
     → 若检测到目标但不满足上述条件：
        - 使用 robot_z 进行 KP 比例前进（裁剪）；
        - 同时根据图像中心偏差调整左右位置（X）和腿高（Y）。

 ⚙️ 控制策略说明：
 - 所有前进控制（除情况 B）均采用比例控制 + 裁剪速度范围
 - 支持动态跟踪目标、精确靠近、失效恢复

===========================================================
"""

import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from detectionresult.msg import DetectionResult

class AutoNavNode(Node):
    def __init__(self):
        super().__init__('diablo_auto_nav_node')

        # 通信接口
        self.publisher = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 2)
        self.subscription = self.create_subscription(
            DetectionResult,
            '/detection_result',
            self.target_callback,
            10)

        # 控制参数
        self.ctrl_msg = MotionCtrl()
        self.kp = 0.5             # 比例控制增益
        self.v_min = 0.05         # 最小前进速度（限制太近时速度不为0）
        self.v_max = 0.5          # 最大前进速度（限制过快撞击）
        self.turn_gain = 0.05     # 左右偏移修正增益
        self.leg_gain = 0.8       # 上下高度修正增益
        self.rotate_speed = 0.1   # 原地旋转速度

        # 检测数据缓存
        self.current_detection = None
        self.target_x_dict = {}
        self.target_y_dict = {}
        self.target_z_dict = {}

        # 定距前进控制（用于情况B）
        self.forward_start_time = None
        self.required_duration = 0.0
        self.is_forwarding = False

        # 控制循环 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    # 工具函数：裁剪前进速度
    def clip_speed(self, speed: float) -> float:
        return max(self.v_min, min(self.v_max, speed))

    # 发布运动控制指令
    def generate_msgs(self, forward: float, left: float, up: float):
        self.ctrl_msg.value.forward = forward
        self.ctrl_msg.value.left = left
        self.ctrl_msg.value.up = up
        self.publisher.publish(self.ctrl_msg)
        self.get_logger().debug(f"CMD: Fwd={forward:.2f}, Lft={left:.2f}, Up={up:.2f}")

    # 停止并关闭节点
    def shutdown_node(self):
        self.get_logger().info("Mission complete. Stopping and shutting down.")
        self.generate_msgs(0.0, 0.0, 0.0)
        self.destroy_node()
        rclpy.shutdown()

    # 主控制逻辑
    def control_loop(self):
        # 1️⃣ 未检测到任何目标，原地旋转
        if (0 not in self.target_z_dict) and (1 not in self.target_z_dict):
            self.get_logger().info("No robot or magnet detected. Rotating.")
            self.generate_msgs(0.0, -self.rotate_speed, 0.0)
            return

        # 获取目标距离
        robot_z = self.target_z_dict.get(1, 0.0)
        magnet_z = self.target_z_dict.get(0, 0.0)

        # 2️⃣ 情况 A：机器人距离 > 0.8m，向机器人前进（比例控制+裁剪）
        if robot_z > 0.8:
            speed = self.clip_speed(self.kp * robot_z)
            self.get_logger().info(f"[A] Robot far ({robot_z:.2f}m). Clipped forward = {speed:.2f} m/s")
            self.generate_msgs(speed, 0.0, 0.0)
            return

        # 3️⃣ 情况 C：机器人靠近，电磁铁已检测，使用磁铁距离前进
        if 0 < robot_z < 1.0 and magnet_z != 0:
            speed = self.clip_speed(self.kp * magnet_z)
            self.get_logger().info(f"[C] Robot@{robot_z:.2f} + Magnet@{magnet_z:.2f} → Clipped forward = {speed:.2f} m/s")
            self.generate_msgs(speed, 0.0, 0.0)
            return

        # 4️⃣ 情况 B：机器人近、电磁铁未检测，固定盲推 5 秒
        if 0 < robot_z < 0.3 and magnet_z == 0:
            self.forward_speed = 0.2
            self.required_duration = 5.0
            self.forward_start_time = self.get_clock().now()
            self.is_forwarding = True
            self.get_logger().info("[B] Too close and no magnet. Moving forward 5s at 0.2 m/s.")
            return

        # 🕒 定时前进过程（情况 B 正在进行）
        if self.is_forwarding:
            elapsed = (self.get_clock().now() - self.forward_start_time).nanoseconds / 1e9
            if elapsed < self.required_duration:
                self.generate_msgs(self.forward_speed, 0.0, 0.0)
            else:
                self.shutdown_node()
            return

        # 5️⃣ fallback 跟踪：左右腿高修正 + 前进（比例控制+裁剪）
        if self.current_detection:
            target_x = self.current_detection.center.x
            target_y = self.current_detection.center.y
            left_cmd = -self.turn_gain * target_x
            up_cmd = self.leg_gain * target_y
            speed = self.clip_speed(self.kp * robot_z) if robot_z > 0 else 0.0
            self.get_logger().info(f"[TRACK] Forward={speed:.2f}, Left={left_cmd:.2f}, Up={up_cmd:.2f}")
            self.generate_msgs(speed, left_cmd, up_cmd)
        else:
            self.generate_msgs(0.0, -self.rotate_speed, 0.0)

    # 目标检测回调：存储目标坐标
    def target_callback(self, msg: DetectionResult):
        self.current_detection = msg
        class_id = msg.class_id
        self.target_x_dict[class_id] = msg.center.x
        self.target_y_dict[class_id] = msg.center.y
        self.target_z_dict[class_id] = msg.center.z

        self.get_logger().info(f"[DETECT] Class {class_id}: x={msg.center.x:.2f}, y={msg.center.y:.2f}, z={msg.center.z:.2f}")

# ROS入口
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
