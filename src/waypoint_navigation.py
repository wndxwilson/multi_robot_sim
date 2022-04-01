#!/usr/bin/env python3
import re
import rospy
import sys
import actionlib
import rospkg
from geometry_msgs.msg import PoseArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger
from multi_robot_sim.srv import MultiRobotSimGoalNodes
from actionlib_msgs.msg import GoalID
from utils.state_machine import StateMachine
from utils.osm_utils import OsmUtil

class WaypointNavigation:
    def __init__(self,namespace):
        """WaypointNavigation
        This module implements the robot way point navigation
        It has 4 states idle, navigation, pause, and end state
        """
        self.namespace = namespace
        self.waypoints = PoseArray()

        path = rospy.get_param("/yaml_path")
        self.gg = OsmUtil(path)

        # Service to pause/start/stop waypoint navigation
        self.start_waypoint_srv = rospy.Service("start_waypoint", Trigger, self.handle_start_waypoint)
        self.stop_waypoint_srv = rospy.Service("stop_waypoint", Trigger, self.handle_stop_waypoint)
        self.pause_waypoint_srv = rospy.Service("pause_waypoint", Trigger, self.handle_pause_waypoint)
        self.send_waypoint_srv = rospy.Service("send_waypoint", MultiRobotSimGoalNodes, self.handle_send_waypoint)

        # Move base client
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("connecting to move base")
        self.move_base_client.wait_for_server()
        rospy.loginfo("connected to move base")

        # Publisher
        self.cancel_pub = rospy.Publisher("move_base/cancel",GoalID,queue_size=10)

        # Params
        self.distance_tolerance = 0.1
        self.robot_status = -1

        # Intial state transition
        self.mode = None 
        self.transition = ""

    def handle_start_waypoint(self,req):
        """
        This service set the transition to start
        """
        self.transition = "resume"
        return (True, "start waypoint navigation")

    def handle_stop_waypoint(self,req):
        """
        This service set the transition to cancel
        """
        self.transition = "cancel"
        return (True, "cancel waypoint navigation")

    def handle_pause_waypoint(self,req):
        """
        This service set the transition to pause
        """
        self.transition = "pause"
        return (True, "Pause waypoint navigation")

    def handle_send_waypoint(self,req):
        """
        This service handles the waypoint navigation
        It formats the waypoints in to goals depending on the mode
        mode0 : 1-way
        mode1 : Disjointed loop
        mode2 : Closed loop
        """
        nodes = list(req.nodes)
        if req.mode == "mode0":
            self.waypoints = self.gg.getGoals(nodes)
            self.mode = "normal"

        elif req.mode == "mode2":
            nodes.append(nodes[0])
            self.waypoints = self.gg.getGoals(nodes)
            self.mode = "loop"

        elif req.mode == "mode1":
            reverse = nodes[:-1][::-1]
            nodes = nodes + reverse
            self.waypoints = self.gg.getGoals(nodes)
            self.mode = "loop"
        
        # print(nodes)
        if(self.waypoints == None):
            self.transition = ""
            return (False)

        self.transition = "navigate"
        return (True)

    def run(self):
        """
        Calls the state machine to run
        """
        self.generateStateMahine().run()
    
    def generateStateMahine(self):
        """
        Initialise the state machine and add all the releveant states
        """
        sm = StateMachine()
        sm.add_state("idleState",self.idleState)
        sm.add_state("NavigationState",self.navigationState)
        sm.add_state("pauseState",self.pauseState)
        sm.add_state("endState",None,end_state=1)
        sm.set_start("idleState")

        return sm

    # State machine states
    def idleState(self):
        """
        This is the idle sate, it waits for the transition "navigate" to go to the navigation state
        """
        if self.transition == "navigate":
            self.transition = ""
            return ("NavigationState")

        else:
            return ("idleState")
    
    def navigationState(self):
        """
        This is the navigation sate, it sends the goals to the waypoint navigation
        """
        if(self.waypoints.poses):
            pose = self.waypoints.poses.pop()
            if self.mode == "loop":
                self.waypoints.poses.insert(0,pose)
            return self.waypointNavigation(pose)

        else:
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            self.transition = ""
            return ("idleState")

    def waypointNavigation(self,pose):
        """
        The waypoint navigation sends the goal to the move base
        if success it will return to the navigations tate
        if paused it will return to the pause state
        if failed or cancel it will return to the idle state
        @parm pose Pose
        @return state
        """
        goal = self.createGoalMsg(pose)
        rospy.logwarn("Going to goal...")
        self.move_base_client.send_goal(goal)

        while(True):
            robot_state = self.move_base_client.get_state()

            if(robot_state in [3,4,5,9]):
                break

            if self.transition == "pause":
                cancel_msg = GoalID()
                self.cancel_pub.publish(cancel_msg)
                self.transition = ""
                self.waypoints.poses.insert(0,goal.target_pose.pose)
                return("pauseState")

            elif self.transition == "cancel":
                cancel_msg = GoalID()
                self.cancel_pub.publish(cancel_msg)
                self.transition = ""
                return("idleState")
        

        return ("navigationState")

        
    def createGoalMsg(self, pose):
        """
        The function converts the Pose msg to MoveBaseGoal msg
        @param pose Pose
        @return goal MoveBaseGoal
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.namespace + "/odom"
        goal.target_pose.pose = pose
        goal.target_pose.header.stamp = rospy.Time.now()

        return goal

    
    def pauseState(self):
        """
        This is the pause sate
        If resume it reutrns to the navgation state
        If cancel it retunrs to the idle state
        @return state
        """
        if self.transition == "resume":
            self.transition = ""
            return("navigationState")
        elif self.transition == "cancel":
            self.transition = ""
            return("idleState")
        return("pauseState")
        
if __name__ == '__main__':
    # initialize node
    rospy.init_node('waypoint_navigation')
    rospy.loginfo("Running waypoint navigation node")

    # Get the arguments
    args = rospy.myargv(argv=sys.argv)
    if(len(args) != 2):
        rospy.logwarn('Namespace name cannot be empty')
        exit()
    namespace = args[1]
    W = WaypointNavigation(namespace)
    W.run()
