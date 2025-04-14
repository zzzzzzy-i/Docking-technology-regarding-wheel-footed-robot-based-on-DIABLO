#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.node import Node

# 导入ROS消息类型（请根据实际情况修改消息类型及包名）
from motion_msgs.msg import MotionCtrl
from ception_msgs.msg import ImuEuler, Imu, BodyState , Battery, Motors

# PyQt5相关模块
from PyQt5 import QtWidgets, QtCore

# 定义信号转发器，包含各个Topic更新的信号
class RosSignalEmitter(QtCore.QObject):
    motionCmdSignal = QtCore.pyqtSignal(str)
    batterySignal   = QtCore.pyqtSignal(str)
    bodyStateSignal = QtCore.pyqtSignal(str)
    imuSignal       = QtCore.pyqtSignal(str)
    imuEulerSignal  = QtCore.pyqtSignal(str)
    motorsSignal    = QtCore.pyqtSignal(str)

# 定义ROS订阅节点
class RosSubscriber(Node):
    def __init__(self, signal_emitter):
        super().__init__('ros_subscriber')
        self.signal_emitter = signal_emitter

        # 订阅上层控制指令（规划速度）
        self.create_subscription(
            MotionCtrl, '/diablo/MotionCmd', 
            self.motion_cmd_callback, 10)
        # 订阅电池状态
        self.create_subscription(
            Battery, '/diablo/sensor/Battery', 
            self.battery_callback, 10)
        # 订阅机器人状态（用于debug）
        self.create_subscription(
            BodyState, '/diablo/sensor/Body_state', 
            self.body_state_callback, 10)
        # 订阅IMU数据（yaw, pitch, roll）
        self.create_subscription(
            Imu, '/diablo/sensor/Imu', 
            self.imu_callback, 10)
        # 订阅IMU Euler数据（加速度、四元数）
        self.create_subscription(
            ImuEuler, '/diablo/sensor/ImuEuler', 
            self.imu_euler_callback, 10)
        # 订阅电机数据（角速度、转速、圈数、位置）
        self.create_subscription(
            Motors, '/diablo/sensor/Motors', 
            self.motors_callback, 10)

    def motion_cmd_callback(self, msg):
        # 将消息转换为字符串（你可以根据消息字段进行更详细的格式化）
        self.signal_emitter.motionCmdSignal.emit(str(msg))

    def battery_callback(self, msg):
        self.signal_emitter.batterySignal.emit(str(msg))

    def body_state_callback(self, msg):
        self.signal_emitter.bodyStateSignal.emit(str(msg))

    def imu_callback(self, msg):
        self.signal_emitter.imuSignal.emit(str(msg))

    def imu_euler_callback(self, msg):
        self.signal_emitter.imuEulerSignal.emit(str(msg))

    def motors_callback(self, msg):
        self.signal_emitter.motorsSignal.emit(str(msg))

# 定义PyQt可视化界面
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Diablo ROS Data Visualizer")
        layout = QtWidgets.QVBoxLayout()

        # 创建用于显示各个Topic数据的Label
        self.motionCmdLabel = QtWidgets.QLabel("MotionCmd: ")
        self.batteryLabel   = QtWidgets.QLabel("Battery: ")
        self.bodyStateLabel = QtWidgets.QLabel("BodyState: ")
        self.imuLabel       = QtWidgets.QLabel("IMU: ")
        self.imuEulerLabel  = QtWidgets.QLabel("IMU Euler: ")
        self.motorsLabel    = QtWidgets.QLabel("Motors: ")

        layout.addWidget(self.motionCmdLabel)
        layout.addWidget(self.batteryLabel)
        layout.addWidget(self.bodyStateLabel)
        layout.addWidget(self.imuLabel)
        layout.addWidget(self.imuEulerLabel)
        layout.addWidget(self.motorsLabel)
        self.setLayout(layout)

    # 定义槽函数，用于接收ROS信号并更新界面
    @QtCore.pyqtSlot(str)
    def update_motion_cmd(self, data):
        self.motionCmdLabel.setText("MotionCmd: " + data)

    @QtCore.pyqtSlot(str)
    def update_battery(self, data):
        self.batteryLabel.setText("Battery: " + data)

    @QtCore.pyqtSlot(str)
    def update_body_state(self, data):
        self.bodyStateLabel.setText("BodyState: " + data)

    @QtCore.pyqtSlot(str)
    def update_imu(self, data):
        self.imuLabel.setText("IMU: " + data)

    @QtCore.pyqtSlot(str)
    def update_imu_euler(self, data):
        self.imuEulerLabel.setText("IMU Euler: " + data)

    @QtCore.pyqtSlot(str)
    def update_motors(self, data):
        self.motorsLabel.setText("Motors: " + data)

if __name__ == '__main__':
    # 初始化ROS2
    rclpy.init()
    # 创建信号转发器
    emitter = RosSignalEmitter()
    # 创建ROS订阅节点，并传入信号转发器
    ros_node = RosSubscriber(emitter)
    # 开启ROS的spin线程（使回调函数能够持续运行）
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # 启动PyQt应用
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()

    # 将ROS信号与界面槽函数连接
    emitter.motionCmdSignal.connect(window.update_motion_cmd)
    emitter.batterySignal.connect(window.update_battery)
    emitter.bodyStateSignal.connect(window.update_body_state)
    emitter.imuSignal.connect(window.update_imu)
    emitter.imuEulerSignal.connect(window.update_imu_euler)
    emitter.motorsSignal.connect(window.update_motors)

    window.show()
    exit_code = app.exec_()

    # 程序退出时清理ROS节点
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)
