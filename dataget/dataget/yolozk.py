import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from detectionresult.msg import DetectionResult

class AutoNavNode(Node):
    def __init__(self):
        # 初始化节点，命名为 'diablo_auto_nav_node'
        super().__init__('diablo_auto_nav_node')
        
        # 创建发布器，用于向 /diablo/MotionCmd 话题发布运动控制消息
        self.publisher = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 2)
        
        # 创建订阅器，接收 /detection_result 话题上的目标检测信息
        self.subscription = self.create_subscription(
            DetectionResult,
            '/detection_result',
            self.target_callback,
            10)
        
        # 初始化运动控制消息对象
        self.ctrl_msg = MotionCtrl()

        # 设置运动控制参数
        self.forward_speed = 0.1     # 前进速度（米/秒）
        self.turn_gain = 0.5         # 转向增益（用于计算左右调整命令）
        self.distance_threshold = 0.2  # 距离阈值：当目标 z 距离小于此值时，认为已到达目标
        self.control_rate = 10       # 控制循环频率（Hz）

        # 新增：用于控制机器人腿长高度的参数，通过 y 坐标计算 up 指令
        self.leg_gain = 0.5        # 该增益决定 y 坐标对腿高控制的影响

        # 保存最新的目标检测数据
        self.current_detection = None

        # 创建定时器，以固定频率调用控制循环函数 control_loop
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

    def generate_msgs(self, forward, left, up):
        """
        将前进、转向和腿高（up）指令写入控制消息，并发布出去。
        
        :param forward: 前进速度（正值前进，负值后退）
        :param left: 转向命令（正值向左转，负值向右转）
        :param up: 控制腿高的指令（例如，正值增加腿长高度，负值降低）
        """
        self.ctrl_msg.value.forward = forward
        self.ctrl_msg.value.left = left
        self.ctrl_msg.value.up = up
        self.publisher.publish(self.ctrl_msg)

    def control_loop(self):
        """
        定时器回调函数，作为运动控制的主循环：
          1. 检查是否已有目标检测数据；
          2. 根据目标的三维坐标计算前进、转向以及腿高调整命令；
          3. 当目标距离足够近时，停止机器人运动。
        """
        # 如果没有检测到目标，则不执行控制命令
        if self.current_detection is None:
            return

        # 提取目标的三维坐标（单位：米）
        target_x = self.current_detection.center.x  # 水平偏差，正值表示目标在右侧
        target_y = self.current_detection.center.y  # 垂直方向数据，用于控制腿高
        target_z = self.current_detection.center.z  # 前后距离，正值表示目标在前方

        # 如果目标前后距离小于阈值，则认为已到达目标，机器人停止运动
        if abs(target_z) < self.distance_threshold:
            self.get_logger().info("Target reached, stopping.")
            self.generate_msgs(0.0, 0.0, 0.0)
            return

        # 根据水平偏差计算转向命令：目标在右侧时，机器人需要向右转（左指令为负值）
        left_command = -self.turn_gain * target_x
        
        # 前进命令保持恒定前进速度
        forward_command = self.forward_speed

        # 根据垂直方向的偏差计算 up 命令，控制机器人的腿长高度
        # 这里采用简单比例控制：up_command = leg_gain * target_y
        up_command = self.leg_gain * target_y

        # 输出当前控制命令和检测数据，便于调试
        self.get_logger().info(
            f"Control: forward={forward_command:.2f}, left={left_command:.2f}, up={up_command:.2f}, "
            f"target_z={target_z:.2f}, target_x={target_x:.2f}, target_y={target_y:.2f}"
        )
        
        # 发布运动控制消息
        self.generate_msgs(forward_command, left_command, up_command)

    def target_callback(self, msg):
        """
        订阅回调函数：每当接收到目标检测数据时调用。
        更新当前检测数据，并打印检测结果以便调试。
        
        :param msg: DetectionResult 消息，包含目标的类别、置信度及中心点的三维坐标
        """
        self.get_logger().info(
            f"Received Detection: Class ID: {msg.class_id}, Confidence: {msg.confidence:.2f}, "
            f"Center: ({msg.center.x:.2f}, {msg.center.y:.2f}, {msg.center.z:.2f})"
        )
        self.current_detection = msg

def main(args=None):
    # 初始化 ROS 2 系统
    rclpy.init(args=args)
    
    # 创建自动导航节点实例
    node = AutoNavNode()
    
    # 进入事件循环，等待订阅消息和定时器回调
    rclpy.spin(node)
    
    # 节点退出后，销毁节点并关闭 ROS 2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
