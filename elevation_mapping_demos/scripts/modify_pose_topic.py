#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2

import numpy as np


def callback(msg):
    global pose_pub
    PoseWithCovarianceStampedMessage = PoseWithCovarianceStamped()
    PoseWithCovarianceStampedMessage.header = msg.header
    PoseWithCovarianceStampedMessage.pose.pose = msg.pose

    # PoseWithCovarianceStampedMessage.pose.covariance = [0]*36
    PoseWithCovarianceStampedMessage.pose.covariance = [0, 0, 0, 0, 0, 0,
                                                        0, 0, 0, 0, 0, 0,
                                                        0, 0, 0, 0, 0, 0,
                                                        0, 0, 0, 0, 0, 0,
                                                        0, 0, 0, 0, 0, 0,
                                                        0, 0, 0, 0, 0, 0]
    # PoseWithCovarianceStampedMessage.header.stamp.secs = time_lidar
    pose_pub.publish(PoseWithCovarianceStampedMessage)





def main():
    global pose_pub

    print("Hello \n")
    rospy.init_node('pose_modifier', anonymous=True)
    rospy.Subscriber("/poseStamped_node", PoseStamped,
                     callback, queue_size=10)

    pose_pub = rospy.Publisher("/poseStamped_with_covariance_node",
                               PoseWithCovarianceStamped, queue_size=100)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
