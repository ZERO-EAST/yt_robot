#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# @Time     : 2023/08/05 下午5:12
# @Author   : jiangyuandong
# @Email    : jyd_wy@163.com
# @File     : remap_imu.py
# @Software : PyCharm

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Quaternion,PoseWithCovarianceStamped


def remap_data(data):
    new_data = Imu()
    new_data = data
    new_data.linear_acceleration.z += 9.8
    remap_imu_pub.publish(new_data)


if __name__ == '__main__':
    rospy.init_node('remap_imu', anonymous=True)
    remap_imu_pub = rospy.Publisher("/imu2", Imu, queue_size=10)
    rospy.Subscriber("/imu", Imu, remap_data, queue_size=5)
    rospy.spin()
