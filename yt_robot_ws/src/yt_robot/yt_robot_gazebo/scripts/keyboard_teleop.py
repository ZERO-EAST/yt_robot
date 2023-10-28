#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# @Time     : 2023/07/14 下午3:20
# @Author   : jiangyuandong
# @Email    : jyd_wy@163.com
# @File     : keyboard_teleop.py
# @Software : PyCharm

import sys
import termios
import tty
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

# 机器人键盘控制
MoveCommands = {"w": (0, 1),  # 前进
                "s": (0, -1),  # 后退
                "a": (1, 0),  # 左转
                "d": (-1, 0),  # 右转
                "q": 1,  # 增大速度系数
                "e": -1,  # 减小速度系数
                "z": "stop",  # 停止
                "x": "return",  # 方向回正
                "c": "exit"  # 退出键盘控制
                }

# 机器人键盘控制info
MoveCommandsInfo = {"w": "forward",
                    "s": "back",
                    "a": "turn left",
                    "d": "turn right",
                    "q": "increase speed gain",
                    "e": "reduce speed gain",
                    "z": "stop",
                    "x": "return",
                    "c": "exit"
                    }


# 限值
def limData(data, max_data):
    if data > 0 and data > max_data:
        data = max_data
    elif data < 0 and abs(data) > max_data:
        data = -max_data
    return data


class RobotTeleop:
    def __init__(self):
        # 键盘控制指令publisher
        self.state_pub = rospy.Publisher("/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)
        # 设置最大参数
        self.max_gain = 5  # 最大速度系数
        self.max_angle = rospy.get_param("max_angle", 0.523)  # 最大转向角
        self.max_speed = rospy.get_param("max_speed", 10)  # 最大速度

        # 当前控制命令状态
        self.current_gain = 1  # 当前速度系数
        self.current_angle = 0  # 当前转向角
        self.current_speed = 0  # 当前速度

        # 显示提示信息
        print(MoveCommandsInfo)

    def publishCommand(self, angle, speed):
        """
        发布指令
        :param angle: 转向角
        :param speed: 速度
        :return:
        """
        ack = AckermannDriveStamped()
        ack.drive.steering_angle = angle
        ack.drive.speed = speed
        self.state_pub.publish(ack)

    def stopRobot(self):
        """
        停止机器人
        :return:
        """
        angle = self.current_angle
        speed = 0
        return angle, speed

    def reRobot(self):
        """
        回正机器人
        :return:
        """
        angle = 0
        speed = self.current_speed
        return angle, speed

    def keyboardLoop(self):
        # 读取按键循环
        while not rospy.is_shutdown():
            # 返回一个包含文件描述符fd的tty属性的列表，用于getkey的第三个参数
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            # 不产生回显效果
            # old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
            try:
                # 改变在所有写入fd引用的对象的输出都被传输后生效，所有已接受但未读入的输入都在改变发生前丢弃(同TCSADRAIN，但会舍弃当前所有值)
                # 将文件描述符 fd 的模式更改为 raw,如果 when 被省略,则默认为 termios.TCSAFLUSH,并传递给 termios.tcsetattr()
                tty.setraw(fd)
                ch = sys.stdin.read(1)
            finally:
                # 改变在所有写入fd的输出都被传输后生效。这个函数应当用于修改影响输出的参数时使用。(当前输出完成时将值改变)
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            # 解析指令
            print("command is: %s " % ch, MoveCommandsInfo[ch])
            if ch in MoveCommands.keys():
                if ch == "q":
                    self.current_gain += 1
                    self.current_gain = min(self.current_gain, self.max_gain)
                    continue
                elif ch == "e":
                    self.current_gain -= 1
                    self.current_gain = max(1, self.current_gain)
                    continue
                elif ch == "z":
                    self.current_speed = 0
                elif ch == "x":
                    self.current_angle = 0
                elif ch == "c":
                    self.publishCommand(0, 0)
                    break
                else:
                    angle_flag, speed_flag = MoveCommands[ch]
                    # 计算
                    self.current_angle += 0.05 * angle_flag
                    self.current_speed += 0.1 * self.current_gain * speed_flag
                    # 限值
                    self.current_angle = limData(self.current_angle, self.max_angle)
                    self.current_speed = limData(self.current_speed, self.max_speed)
            else:
                print("command is: %s  未定义" % ch)
                continue

            # 发送消息
            self.publishCommand(self.current_angle, self.current_speed)
            print("    current angle: ", self.current_angle)
            print("    current speed: ", self.current_speed)
            print("\n")


if __name__ == "__main__":
    rospy.init_node("keyboard_teleop")
    try:
        ControlDemo = RobotTeleop()
        ControlDemo.keyboardLoop()
    except rospy.ROSInterruptException:
        pass
