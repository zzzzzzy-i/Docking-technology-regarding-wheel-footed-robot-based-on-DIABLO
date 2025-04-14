#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
#from scipy.spatial.transform import Rotation as R  # 使用 scipy 替代 tf_transformations

class RobotInfoListener(Node):
    def __init__(self):
        super().__init__('robot_info_listener')
        # 订阅各个Topic
        self.create_subscription(String, '/diablo/MotionCmd', self.motion_cmd_callback, 10)
        self.create_subscription(String, '/diablo/sensor/Battery', self.battery_callback, 10)
        self.create_subscription(String, '/diablo/sensor/Body_state', self.body_state_callback, 10)
        self.create_subscription(Imu, '/diablo/sensor/Imu', self.imu_callback, 10)
        self.create_subscription(String, '/diablo/sensor/ImuEuler', self.imu_euler_callback, 10)
        self.create_subscription(String, '/diablo/sensor/Motors', self.motors_callback, 10)
        self.get_logger().info("机器人信息监听节点已启动")

    def motion_cmd_callback(self, msg):
        self.get_logger().info(f"收到 MotionCmd 数据: {msg.data}")

    def battery_callback(self, msg):
        self.get_logger().info(f"收到 Battery 数据: {msg.data}")

    def body_state_callback(self, msg):
        self.get_logger().info(f"收到 Body_state 数据: {msg.data}")

    #def imu_callback(self, msg):
        # 使用 scipy 将四元数转换为欧拉角（roll, pitch, yaw）
       # quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
       # r = R.from_quat(quaternion)
       # roll, pitch, yaw = r.as_euler('xyz', degrees=False)  # 返回弧度值
       # self.get_logger().info(f"收到 Imu 数据: yaw: {yaw:.2f}, pitch: {pitch:.2f}, roll: {roll:.2f}")

   # def imu_euler_callback(self, msg):
     #   self.get_logger().info(f"收到 ImuEuler 数据: {msg.data}")

    def motors_callback(self, msg):
        self.get_logger().info(f"收到 Motors 数据: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotInfoListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点停止运行")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
