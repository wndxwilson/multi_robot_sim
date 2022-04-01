#!/usr/bin/env python
import rospy
import tf
import sys
from geometry_msgs.msg import PoseStamped, Pose

class RobotPosePublisher:
    def __init__(self,namespace):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        base_link = namespace +'/base_link'

        self.pub = rospy.Publisher('robot_pose', PoseStamped, queue_size=10)

        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/map', base_link, rospy.Time(0))
                
                pose = PoseStamped()
                pose.header.frame_id = '/map'
                pose.header.stamp = rospy.Time.now()

                pose.pose.position.x = trans[0]
                pose.pose.position.y = trans[1]
                pose.pose.position.z = trans[2]

                pose.pose.orientation.x = rot[0]
                pose.pose.orientation.y = rot[1]
                pose.pose.orientation.z = rot[2]
                pose.pose.orientation.w = rot[3]

                self.pub.publish(pose)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

if __name__ == '__main__':
    # initialize node
    rospy.init_node('robot_pose_publisher')
    rospy.loginfo("Running robot pose publisher node")

    # Get the arguments
    args = rospy.myargv(argv=sys.argv)
    if(len(args) != 2):
        rospy.logwarn('need to define tf for baselink')
        exit()
    namespace = args[1]
    R = RobotPosePublisher(namespace)
