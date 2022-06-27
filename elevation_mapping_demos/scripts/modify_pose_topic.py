#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np


def callback(msg):
    PoseWithCovarianceStampedMessage = PoseWithCovarianceStamped()
    PoseWithCovarianceStampedMessage.header = msg.header
    PoseWithCovarianceStampedMessage.pose.pose = msg.pose
    PoseWithCovarianceStampedMessage.pose.covariance = np.ones((4, 4))*0.005


def main():
    rospy.init_node('pose_modifier', anonymous=True)
    rospy.Subscriber("/poseStamped_node", PoseStamped,
                     callback, queue_size=1)
    rospy.Publisher("/poseStamped_with_covariance_node",
                    PoseWithCovarianceStamped, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
