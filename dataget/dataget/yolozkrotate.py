"""
自动导航节点（完成目标后自动关闭版本）
功能：
1. 检测到目标时进行跟踪控制（转向+腿高调整）
2. 目标距离过近时执行定距前进
3. 前进完成后自动关闭节点
4. 丢失目标时原地旋转
"""

import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from detectionresult.msg import DetectionResult

class AutoNavNode(Node):
    def __init__(self):
        super().__init__('diablo_auto_nav_node')
        
        # ============= 通信接口初始化 =============
        self.publisher = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 2)
        self.subscription = self.create_subscription(
            DetectionResult,
            '/detection_result',
            self.target_callback,
            10)

        # ============= 控制参数初始化 =============
        self.ctrl_msg = MotionCtrl()
        self.forward_speed = 0.05      # 前进速度 (m/s)
        self.turn_gain = 0.05          # 转向增益
        self.distance_threshold = 0.6  # 触发距离 (m)
        self.leg_gain = 0.8            # 腿高调整增益
        self.rotate_speed = 0.1        # 旋转速度 (左指令负值=右转)
        self.current_detection = None  # 当前检测目标

        # ============= 定距前进控制参数 =============
        self.forward_start_time = None  # 前进开始时间
        self.required_duration = 0.0    # 需要前进的持续时间
        self.is_forwarding = False      # 前进状态标志

        # ============= 启动控制循环 =============
        self.timer = self.create_timer(1.0/10, self.control_loop)

    def generate_msgs(self, forward: float, left: float, up: float):
        """生成并发布运动控制指令"""
        self.ctrl_msg.value.forward = forward
        self.ctrl_msg.value.left = left
        self.ctrl_msg.value.up = up
        self.publisher.publish(self.ctrl_msg)
        self.get_logger().debug(f"CMD: Fwd={forward:.2f}, Lft={left:.2f}, Up={up:.2f}")

    def shutdown_node(self):
        """安全关闭节点的自定义方法"""
        self.get_logger().info("Mission complete, shutting down node...")
        # 发送停止指令确保机器人静止
        self.generate_msgs(0.0, 0.0, 0.0)
        # 销毁节点并关闭ROS
        self.destroy_node()
        rclpy.shutdown()

    def control_loop(self):
        """主控制循环"""
        # 情况1：未检测到目标
        if self.current_detection is None:
            self.get_logger().info("No target: Rotating clockwise")
            self.generate_msgs(0.0, -self.rotate_speed, 0.0)
            return

        # 情况2：正在执行定距前进
        if self.is_forwarding:
            elapsed = (self.get_clock().now() - self.forward_start_time).nanoseconds / 1e9
            if elapsed < self.required_duration:
                self.generate_msgs(self.forward_speed, 0.0, 0.0)
                return
            else:
                self.shutdown_node()  # 关键修改：完成后关闭节点
                return

        # 情况3：目标进入触发距离
        target_z = self.current_detection.center.z
        if abs(target_z) < self.distance_threshold:
            required_distance = self.distance_threshold - 0.22
            self.required_duration = required_distance / self.forward_speed
            
            self.forward_start_time = self.get_clock().now()
            self.is_forwarding = True
            self.get_logger().info(
                f"Starting {required_distance:.2f}m move "
                f"({self.required_duration:.2f}s)"
            )
            return

        # 情况4：正常跟踪控制
        target_x = self.current_detection.center.x
        target_y = self.current_detection.center.y
        left_cmd = -self.turn_gain * target_x
        up_cmd = self.leg_gain * target_y
        self.generate_msgs(self.forward_speed, left_cmd, up_cmd)

    def target_callback(self, msg: DetectionResult):
        """目标检测回调函数"""
        self.current_detection = msg
        self.get_logger().info(
            f"Detected: Z={msg.center.z:.2f}m | "
            f"X={msg.center.x:.2f}m | Y={msg.center.y:.2f}m"
        )

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
