#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import numpy as np
import math

class ImuPitchRollEstimator:
    def __init__(self):
        rospy.init_node('imu_pitch_roll_estimator', anonymous=True)
        # TODO(): 修改为你当前飞机的IMU话题
        self.sub = rospy.Subscriber('/x280_1/livox/imu', Imu, self.imu_callback)
        self.pitch = 0.0
        self.roll = 0.0
        self.alpha = 0.02  # 低通滤波系数
        rospy.loginfo("IMU Pitch-Roll Estimator (ZYX) started")
        rospy.spin()

    def imu_callback(self, msg: Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # 静止假设，Yaw = 0
        pitch_measured = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        roll_measured  = math.atan2(ay, az)

        # 低通滤波平滑
        self.pitch = (1 - self.alpha) * self.pitch + self.alpha * pitch_measured
        self.roll  = (1 - self.alpha) * self.roll + self.alpha * roll_measured
        # TODO: 因为雷达是反向安装的，所以Yaw固定180度
        yaw = math.pi
        # yaw = 176 * math.pi / 180

        # ZYX旋转矩阵
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw),  math.cos(yaw), 0],
                       [0, 0, 1]])
        Ry = np.array([[math.cos(self.pitch), 0, math.sin(self.pitch)],
                       [0, 1, 0],
                       [-math.sin(self.pitch), 0, math.cos(self.pitch)]])
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(self.roll), -math.sin(self.roll)],
                       [0, math.sin(self.roll),  math.cos(self.roll)]])

        R = Rz @ Ry @ Rx  # ZYX顺序

        # rospy.loginfo("Pitch: {:.3f} deg, Roll: {:.3f} deg".format(
        #     math.degrees(self.pitch), math.degrees(self.roll)))
        # rospy.loginfo("Rotation matrix (ZYX):\n{}".format(R))

        R_inv = np.linalg.inv(R)
        print("\n Rotation matrix (ZYX):")
        print("[%.8e, %.8e, %.8e," % tuple(R_inv[0]))
        print(" %.8e, %.8e, %.8e," % tuple(R_inv[1]))
        print(" %.8e, %.8e, %.8e]" % tuple(R_inv[2]))


if __name__ == '__main__':
    try:
        ImuPitchRollEstimator()
    except rospy.ROSInterruptException:
        pass