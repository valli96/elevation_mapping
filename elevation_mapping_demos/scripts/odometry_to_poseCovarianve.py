import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def odom_callback(msg, arg):
    publisher = arg
    new_message = PoseWithCovarianceStamped()
    new_message.header = msg.header
    new_message.pose = msg.pose
    publisher.publish(new_message)
if __name__ == '__main__':
    rospy.init_node('odom_to_pose')

    topic_name = rospy.get_param('~odom_topic', '')
    publisher_name = rospy.get_param('~pub_topic_name', '')
    # pose_name = str(publisher_name) + "_pose"
    if topic_name == '' or publisher_name == '':
        rospy.logerr("Couldn't get subscriber's topic name or publisher's topic name . Exiting...")
    else:
        publisher = rospy.Publisher(publisher_name, PoseWithCovarianceStamped, queue_size=10)
        rospy.Subscriber(topic_name, Odometry, odom_callback, publisher)
        while not rospy.is_shutdown():
            pass