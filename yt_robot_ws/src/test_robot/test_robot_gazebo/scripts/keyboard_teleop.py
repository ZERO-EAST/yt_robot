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


def keyboardLoop():
    rospy.init_node("keyboard_teleop")
    state_pub = rospy.Publisher("/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=10)

    def publishCommand(angle, speed):
        ack = AckermannDriveStamped()
        ack.drive.steering_angle = angle
        ack.drive.speed = speed
        state_pub.publish(ack)

    def stop_robot():
        ack = AckermannDriveStamped()
        ack.drive.steering_angle = 0
        ack.drive.speed = 0
        state_pub.publish(ack)

    # 显示提示信息
    print("Reading from keyboard")
    print("Use WASD keys to control the robot")
    print("Press Caps to move faster")
    print("Press q to quit")
    current_angle = 0
    current_velocity = 0

    # 读取按键循环
    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        # 不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("command is: ", ch)
        if ch == 'w':
            speed_flag = 1
            turn_flag = 0
        elif ch == 's':
            speed_flag = -1
            turn_flag = 0
        elif ch == 'a':
            speed_flag = 0
            turn_flag = 1
        elif ch == 'd':
            speed_flag = 0
            turn_flag = -1
        elif ch == 'W':
            speed_flag = 1
            turn_flag = 0
        elif ch == 'S':
            speed_flag = -1
            turn_flag = 0
        elif ch == 'A':
            speed_flag = 0
            turn_flag = 1
        elif ch == 'D':
            speed_flag = 0
            turn_flag = -1
        elif ch == 'q':
            exit()
        else:
            speed_flag = 0
            turn_flag = 0

        # 发送消息
        current_angle += 0.05 * turn_flag
        current_velocity += 0.1 * speed_flag
        publishCommand(current_angle, current_velocity)

    # 停止机器人
    stop_robot()


if __name__ == '__main__':
    try:
        max_angle = rospy.get_param('max_angle', 0.8)
        max_velocity = rospy.get_param('max_velocity', 1.0)
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
