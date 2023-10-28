#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# @Time     : 2023/07/14 下午3:12
# @Author   : jiangyuandong
# @Email    : jyd_wy@163.com
# @File     : keyboard_teleop.py
# @Software : PyCharm

import math
import time
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64




# 接受键盘消息控制小车的速度和角度
def set_throttle_steer(data):
    """
    header: 
        seq: 1
        stamp: 
            secs: 0
            nsecs: 0
            frame_id: ''
    drive: 
        steering_angle: 0.0
        steering_angle_velocity: 0.0
        speed: 2.0
        acceleration: 0.0
        jerk: 0.0

    float32 steering_angle // 转向角
    float32 steering_angle_velocity // 转向角的速度,不需要设置
    float32 speed // 向前的速度
    float32 acceleration // 加速度
    float32 jerk // 变加速度
    """
    angle = data.drive.steering_angle
    velocity = data.drive.speed

    abs_angle = abs(angle)
    abs_velocity = abs(velocity)

    # 左转大于0，右转小于0，前进大于0，后退小于0
    # 运动模型计算
    if angle > 0:
        angle_left = math.atan2(axis_l, axis_l / math.tan(abs_angle) - front_wheel_d)
        angle_right = math.atan2(axis_l, axis_l / math.tan(abs_angle) + front_wheel_d)
        velocity_left = abs_velocity - abs_velocity * math.tan(abs_angle) / axis_l * back_wheel_d
        velocity_right = abs_velocity + abs_velocity * math.tan(abs_angle) / axis_l * back_wheel_d
        if velocity < 0:
            velocity_left = -velocity_left
            velocity_right = -velocity_right

    elif angle < 0:
        angle_left = -abs(math.atan2(axis_l, axis_l / math.tan(abs_angle) + front_wheel_d))
        angle_right = -abs(math.atan2(axis_l, axis_l / math.tan(abs_angle) - front_wheel_d))
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
        data = max_data
    return data


# 转换cmd_vel的Twist消息为阿克曼模型的四个轮子的速度和前轮角度
def set_speed(data):
    # global flag_move
    pub_pos_left_bridge_wheel = rospy.Publisher("/test_robot/left_bridge_position_controller/command", Float64,
                                                queue_size=1)
    pub_pos_right_bridge_wheel = rospy.Publisher("/test_robot/right_bridge_position_controller/command", Float64,
                                                 queue_size=1)
    # pub_vel_left_front_wheel = rospy.Publisher('/test_robot/left_front_wheel_velocity_controller/command', Float64,
    #                                            queue_size=1)
    # pub_vel_right_front_wheel = rospy.Publisher('/test_robot/right_front_wheel_velocity_controller/command', Float64,
    #                                             queue_size=1)
    pub_vel_left_back_wheel = rospy.Publisher("/test_robot/left_back_velocity_controller/command", Float64,
                                              queue_size=1)
    pub_vel_right_back_wheel = rospy.Publisher("/test_robot/right_back_velocity_controller/command", Float64,
                                               queue_size=1)

    # Velocity is in terms of radians per second.
    # Want to go 1 m/s with a wheel of radius 0.05m. This translates to 19.97 radians per second, roughly 20.
    # However, at a multiplication factor of 20 speed is half of what it should be, so doubled to 40.
    x = data.linear.x
    z = data.angular.z
    axis_distance = 0.335  # 轴距
    wheel_distance = 0.305  # 两侧轮子之间的距离
    if z != 0 and x != 0:
        r = math.fabs(x / z)  # 转弯半径（车子中心到转弯的圆心）

        r_left_back = r - (math.copysign(1, z) * (wheel_distance / 2.0))  # r为小车中心的转弯半径，所以T需要除以2在叠加上去
        r_right_back = r + (math.copysign(1, z) * (wheel_distance / 2.0))
        r_left_front = math.sqrt(math.pow(r_left_back, 2) + math.pow(axis_distance, 2))
        r_right_front = math.sqrt(math.pow(r_right_back, 2) + math.pow(axis_distance, 2))
        v_left_back = x * r_left_back / r
        v_right_back = x * r_right_back / r
        v_left_front = x * r_left_front / r
        v_right_front = x * r_right_front / r
        angle_left_front = math.atan2(axis_distance, r_left_front) * math.copysign(1, z)
        angle_right_front = math.atan2(axis_distance, r_right_front) * math.copysign(1, z)

    else:
        v_left_back = x
        v_right_back = x
        v_left_front = x
        v_right_front = x
        angle_left_front = z
        angle_right_front = z
    angle_left_front = limSteer(angle_left_front, 0.7)  # 最大转弯角度的弧度为0.7
    angle_right_front = limSteer(angle_right_front, 0.7)

    pub_pos_left_bridge_wheel.publish(angle_left_front)
    pub_pos_right_bridge_wheel.publish(angle_right_front)
    # pub_vel_left_front_wheel.publish(v_left_front * 100)
    # pub_vel_right_front_wheel.publish(v_right_front * 100)
    pub_vel_left_back_wheel.publish(v_left_back * 100)
    pub_vel_right_back_wheel.publish(v_right_back * 100)


def servo_commands():
    rospy.init_node('servo_commands', anonymous=True)
    rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)
    rospy.spin()


if __name__ == '__main__':
    axis_l = 0.25  # 轴距
    front_wheel_d = 0.16 / 2.0  # 前轮轮距
    back_wheel_d = 0.16 / 2.0  # 后轮轮距
    wheel_radius = 0.025  # 轮胎半径

    pub_pos_left_bridge_wheel = rospy.Publisher("/test_robot/left_bridge_position_controller/command", Float64,
                                                queue_size=1)
    pub_pos_right_bridge_wheel = rospy.Publisher("/test_robot/right_bridge_position_controller/command", Float64,
                                                 queue_size=1)
    pub_vel_left_back_wheel = rospy.Publisher("/test_robot/left_back_velocity_controller/command", Float64,
                                              queue_size=1)
    pub_vel_right_back_wheel = rospy.Publisher("/test_robot/right_back_velocity_controller/command", Float64,
                                               queue_size=1)
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
