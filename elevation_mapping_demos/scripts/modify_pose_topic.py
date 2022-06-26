#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import numpy as np


def callback(msg):
    PoseWithCovarianceStampedMessage = geometry_msgs.msg.PoseWithCovarianceStamped()
    PoseWithCovarianceStampedMessage.header = msg.header
    PoseWithCovarianceStampedMessage.pose.pose = msg.pose
    PoseWithCovarianceStampedMessage.pose.covariance = np.ones((4, 4))*0.005


def main():
    rospy.Subscriber("/poseStamped_node", geometry_msgs,
                     callback, queue_size=1)
    rospy.Publisher("/poseStamped_with_covarwiance_node",
                    geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
