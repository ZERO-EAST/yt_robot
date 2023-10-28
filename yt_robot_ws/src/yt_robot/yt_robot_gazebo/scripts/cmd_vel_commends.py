#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# @Time     : 2023/07/25 下午3:12
# @Author   : jiangyuandong
# @Email    : jyd_wy@163.com
# @File     : cmd_vel_commends.py
# @Software : PyCharm

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time

# 转换cmd_vel的Twist消息为阿克曼模型的四个轮子的速度和前轮角度
def set_speed(data):
    if not data.linear.x == 0:
        angle = math.atan(data.angular.z * axis_l / data.linear.x)
    else:
        angle = 0
    velocity = data.linear.x

    abs_angle = abs(angle)
    abs_velocity = abs(velocity)

    # 左转大于0，右转小于0，前进大于0，后退小于0
    # 运动模型计算
    if abs_angle > 0.05:
        if angle > 0:
            angle_left = math.atan2(axis_l, axis_l / math.tan(abs_angle) - front_wheel_d)
            angle_right = math.atan2(axis_l, axis_l / math.tan(abs_angle) + front_wheel_d)
            angle_left = limSteer(angle_left, max_angle)
            angle_right = limSteer(angle_right, max_angle)
            velocity_left = abs_velocity - abs_velocity * math.tan(abs_angle) / axis_l * back_wheel_d
            velocity_right = abs_velocity + abs_velocity * math.tan(abs_angle) / axis_l * back_wheel_d
            if velocity < 0:
                velocity_left = -velocity_left
                velocity_right = -velocity_right

        elif angle < 0:
            angle_left = -abs(math.atan2(axis_l, axis_l / math.tan(abs_angle) + front_wheel_d))
            angle_right = -abs(math.atan2(axis_l, axis_l / math.tan(abs_angle) - front_wheel_d))
            angle_left = limSteer(angle_left, max_angle)
            angle_right = limSteer(angle_right, max_angle)
            velocity_left = abs_velocity + abs_velocity * math.tan(abs_angle) / axis_l * back_wheel_d
            velocity_right = abs_velocity - abs_velocity * math.tan(abs_angle) / axis_l * back_wheel_d
            if velocity < 0:
                velocity_left = -velocity_left
                velocity_right = -velocity_right
    else:
        angle_left = 0
        angle_right = 0

        velocity_left = velocity
        velocity_right = velocity
    # time.sleep(0.5)
    print("command angle left", angle_left)
    print("command angle right", angle_right)
    print("command velocity left", velocity_left)
    print("command velocity right", velocity_right)
    print("\n")
    pub_pos_left_bridge_wheel.publish(angle_left)
    pub_pos_right_bridge_wheel.publish(angle_right)
    pub_vel_left_back_wheel.publish(velocity_left / wheel_radius)
    pub_vel_right_back_wheel.publish(velocity_right / wheel_radius)


# 设置最大的角度
def limSteer(data, max_data):
    if data > 0 and data > max_data:
        data = max_data
    elif data < 0 and math.fabs(data) > max_data:
        data = -max_data
    return data


def servo_commands():
    rospy.init_node('cmd_vel_commands', anonymous=True)
    rospy.Subscriber("/remap_cmd_vel", Twist, set_speed, queue_size=5)
    # rospy.Subscriber("/cmd_vel", Twist, set_speed, queue_size=5)
    rospy.spin()


if __name__ == '__main__':
    axis_l = 1.4  # 轴距
    front_wheel_d = 0.566 / 2.0  # 前轮轮距
    back_wheel_d = 0.77 / 2.0  # 后轮轮距
    wheel_radius = 0.2521  # 轮胎半径
    max_speed = 10.0  # 最大速度
    max_angle = 0.523  # 最大转角

    pub_pos_left_bridge_wheel = rospy.Publisher("/yt_robot/left_bridge_position_controller/command", Float64,
                                                queue_size=1)
    pub_pos_right_bridge_wheel = rospy.Publisher("/yt_robot/right_bridge_position_controller/command", Float64,
                                                 queue_size=1)
    pub_vel_left_back_wheel = rospy.Publisher("/yt_robot/left_back_velocity_controller/command", Float64,
                                              queue_size=1)
    pub_vel_right_back_wheel = rospy.Publisher("/yt_robot/right_back_velocity_controller/command", Float64,
                                               queue_size=1)
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
