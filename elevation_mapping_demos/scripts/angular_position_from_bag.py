#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler

# import numpy as np


angular_pose = [0.0, 0.0, 0.0]
time_last_msg = 0
angular_pose_quaternion = [0,0,0,0]

def callback_integration(msg):
    global time_last_msg, angular_pose_quaternion
    time_diff = (msg.header.stamp.nsecs/pow(10, 6)- time_last_msg)
    angular_pose[0] = (angular_pose[0] +
                       (msg.angular_velocity.x * time_diff/pow(10, 3))) % 360
    angular_pose[1] = (angular_pose[1] +
                       (msg.angular_velocity.y * time_diff/pow(10, 3))) % 360
    angular_pose[2] = (angular_pose[2] +
                       (msg.angular_velocity.z * time_diff/pow(10, 3))) % 360
    time_last_msg = msg.header.stamp.nsecs/pow(10, 6)

    ## in quartanicons 
    # print(angular_pose)
    angular_pose_quaternion = quaternion_from_euler(angular_pose[0],angular_pose[1],angular_pose[2])
    

def callback_pose(msg):
    global angular_pose_quaternions
    global pose_pub

    msg.pose.pose.orientation.z = angular_pose_quaternion[0]
    msg.pose.pose.orientation.y = angular_pose_quaternion[1]
    msg.pose.pose.orientation.x = angular_pose_quaternion[2]
    msg.pose.pose.orientation.w = angular_pose_quaternion[3]
    pose_pub.publish(msg)
    



def main():
    global time_last_msg
    global pose_pub
    rospy.init_node('angular_integrator', anonymous=True)
    rospy.Subscriber("/os1_cloud_node/imu", Imu,
                     callback_integration, queue_size=1)
    rospy.Subscriber("/poseStamped_with_covariance_node", PoseWithCovarianceStamped,
                    callback_pose, queue_size=1)
    pose_pub = rospy.Publisher("/poseStamped_with_covariance_node_2",
                               PoseWithCovarianceStamped, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
