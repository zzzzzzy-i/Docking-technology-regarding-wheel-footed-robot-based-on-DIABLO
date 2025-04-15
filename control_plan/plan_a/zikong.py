#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl

ctrlMsgs = MotionCtrl()

def generMsgs(forward=None, left=None, roll=None, up=None,
              pitch=None, mode_mark=False, height_ctrl_mode=None,
              pitch_ctrl_mode=None, roll_ctrl_mode=None, stand_mode=None,
              jump_mode=False, dance_mode=None):
    global ctrlMsgs
    ctrlMsgs.mode_mark = mode_mark
    ctrlMsgs.mode.jump_mode = jump_mode

    if dance_mode is not None:
        ctrlMsgs.mode.split_mode = dance_mode
    if forward is not None:
        ctrlMsgs.value.forward = forward
    if left is not None:
        ctrlMsgs.value.left = left
    if pitch is not None:
        ctrlMsgs.value.pitch = pitch
    if roll is not None:
        ctrlMsgs.value.roll = roll
    if up is not None:
        ctrlMsgs.value.up = up
    if height_ctrl_mode is not None:
        ctrlMsgs.mode.height_ctrl_mode = height_ctrl_mode
    if pitch_ctrl_mode is not None:
        ctrlMsgs.mode.pitch_ctrl_mode = pitch_ctrl_mode
    if roll_ctrl_mode is not None:
        ctrlMsgs.mode.roll_ctrl_mode = roll_ctrl_mode
    if stand_mode is not None:
        ctrlMsgs.mode.stand_mode = stand_mode

def main(args=None):
    global ctrlMsgs
    rclpy.init(args=args)
    node = Node("diablo_auto_node")
    teleop_cmd = node.create_publisher(MotionCtrl, "diablo/MotionCmd", 2)

    print("机器人进入站立模式...")
    generMsgs(mode_mark=True, stand_mode=True)
    teleop_cmd.publish(ctrlMsgs)
    time.sleep(2.0)  # 等待站立完成

    print("机器人向前移动1米...")
    generMsgs(forward=1.0)
    teleop_cmd.publish(ctrlMsgs)
    time.sleep(1.0)  # 持续1秒向前运动

    print("停止运动")
    generMsgs(forward=0.0)
    teleop_cmd.publish(ctrlMsgs)

    print("动作完成，退出程序")
    rclpy.shutdown()

if __name__ == "__main__":
    main()

