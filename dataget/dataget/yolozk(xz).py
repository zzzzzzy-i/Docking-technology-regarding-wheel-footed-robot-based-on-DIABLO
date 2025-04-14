import rclpy
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from ception_msgs.msg import DetectionResult

class AutoNavNode(Node):
    def __init__(self):
        # 初始化节点，命名为 'diablo_auto_nav_node'
        super().__init__('diablo_auto_nav_node')
        
        # 创建发布器，用于向 /diablo/MotionCmd 话题发布运动控制消息（MotionCtrl 类型）
        self.publisher = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 2)
        
        # 创建订阅器，接收 /detection_result 话题上的目标检测信息（DetectionResult 类型）
        self.subscription = self.create_subscription(
            DetectionResult,
            '/detection_result',
            self.target_callback,
            10)
        
        # 初始化运动控制消息对象，用于后续复用
        self.ctrl_msg = MotionCtrl()

        # 设置运动控制参数
        self.forward_speed = 0.1     # 前进速度（米/秒）
        self.turn_gain = 0.5         # 转向增益，用于计算左右调整指令
        self.distance_threshold = 0.2  # 距离阈值，当目标前向距离小于此值时停止运动
        self.control_rate = 10       # 控制循环频率（Hz）

        # 当前最新的目标检测数据，初始为空
        self.current_detection = None

        # 创建定时器，以固定频率调用控制循环函数 control_loop
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

    def generate_msgs(self, forward, left):
        """
        将前进和转向指令写入控制消息，并通过发布器发送出去。
        
        :param forward: 前进指令（正值为前进，负值为后退）
        :param left: 转向指令（正值表示向左转，负值表示向右转）
        """
        self.ctrl_msg.value.forward = forward
        self.ctrl_msg.value.left = left
        self.publisher.publish(self.ctrl_msg)

    def control_loop(self):
        """
        定时器回调函数，作为运动控制的主循环：
          1. 检查是否有最新的目标检测数据；
          2. 根据目标的相对三维坐标计算前进和转向命令；
          3. 如果目标距离足够近，则停止机器人运动。
        """
        # 如果还未接收到目标检测数据，则不执行控制逻辑
        if self.current_detection is None:
            return

        # 从检测数据中提取目标的相对坐标
        target_x = self.current_detection.center.x  # 水平偏移（正值表示目标在机器人右侧）
        target_z = self.current_detection.center.z  # 前后距离（正值表示目标在机器人前方）

        # 如果目标距离小于阈值，认为已到达目标，发送停止指令
        if abs(target_z) < self.distance_threshold:
            self.get_logger().info("Target reached, stopping.")
            self.generate_msgs(0.0, 0.0)
            return

        # 计算转向指令：采用简单比例控制，目标在右侧时（target_x > 0）需要向右转
        # 由于机器人控制中正的 left 指令表示向左转，所以这里取负值
        left_command = -self.turn_gain * target_x

        # 前进命令保持固定前进速度
        forward_command = self.forward_speed

        # 输出当前控制命令和目标信息，便于调试
        self.get_logger().info(
            f"Control: forward={forward_command:.2f}, left={left_command:.2f}, "
            f"target_z={target_z:.2f}, target_x={target_x:.2f}"
        )
        # 发布运动控制消息
        self.generate_msgs(forward_command, left_command)

    def target_callback(self, msg):
        """
        订阅器的回调函数：每次接收到目标检测数据时调用
        更新当前检测数据，并输出检测结果信息以便调试。
        
        :param msg: DetectionResult 消息，包含目标的类别、置信度及中心三维坐标
        """
        self.get_logger().info(
            f"Received Detection: Class ID: {msg.class_id}, Confidence: {msg.confidence:.2f}, "
            f"Center: ({msg.center.x:.2f}, {msg.center.y:.2f}, {msg.center.z:.2f})"
        )
        # 保存最新的目标检测信息，供控制循环使用
        self.current_detection = msg

def main(args=None):
    # 初始化 ROS 2 系统
    rclpy.init(args=args)
    
    # 创建自动导航节点实例
    node = AutoNavNode()
    
    # 进入事件循环，等待订阅回调和定时器调用
    rclpy.spin(node)
    
    # 节点退出后清理资源
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
