#!/usr/bin/env python
import rospy
import message_filters
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from multi_robot_sim.msg import MultiRobotSimStatus
import sys

class StatusRelay:
    def __init__(self,namespace):
        self.namespace = namespace

        pose_sub = message_filters.Subscriber("odom",Odometry)
        status_sub = message_filters.Subscriber("move_base/status", GoalStatusArray)

        ts = message_filters.ApproximateTimeSynchronizer([pose_sub,status_sub],10,0.1,allow_headerless=True)

        ts.registerCallback(self.sync_callback)

        self.status_sender = rospy.Publisher('/status', MultiRobotSimStatus, queue_size = 20)

    def sync_callback(self,pose,status):
        status_msg = MultiRobotSimStatus()
        status_msg.robot_id = self.namespace
        status_msg.position = pose.pose.pose.position
        if(len(status.status_list)):
            status_msg.status = status.status_list[-1].status
        else:
            status_msg.status = 0

        self.status_sender.publish(status_msg)

if __name__ == '__main__':
    # initialize node
    rospy.init_node('status_relay')
    rospy.loginfo("Running status relay node")

    # Get the arguments
    args = rospy.myargv(argv=sys.argv)
    if(len(args) != 2):
        rospy.logwarn('need to define namespace')
        exit()
    namespace = args[1]
    S = StatusRelay(namespace)
    rospy.spin()
