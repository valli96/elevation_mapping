#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

import numpy as np


angular_pose = [0.0, 0.0, 0.0]
time_last_msg = 0


def callback_integration(msg):
    global time_last_msg
    time_diff = (msg.header.stamp.nsecs - time_last_msg)
    angular_pose[0] = (angular_pose[0] +
                       (msg.angular_velocity.x * time_diff/pow(10, 6))) % 360
    angular_pose[1] = (angular_pose[1] +
                       (msg.angular_velocity.y * time_diff/pow(10, 6))) % 360
    angular_pose[2] = (angular_pose[2] +
                       (msg.angular_velocity.z * time_diff/pow(10, 6))) % 360
    time_last_msg = msg.header.stamp.nsecs
    print(angular_pose)


def main():
    global time_last_msg
    rospy.init_node('angular_integrator', anonymous=True)
    rospy.Subscriber("/os1_cloud_node/imu", Imu,
                     callback_integration, queue_size=1)

    # pose_pub = rospy.Publisher("/poseStamped_with_covariance_node",
    #                            PoseWithCovarianceStamped, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
