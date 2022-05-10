# !/usr/local/bin/python3

import networkx as nx
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import yaml
import os 
import math

class OsmUtil():
    """OsmUtil
    This module contians the graph file, center point and scale
    It maps the gps coordinate to xy coorindates
    """

    def __init__(self,path):
        """
        @param path String
        Retrives the node graph, center reference point and scale from the path
        """
        parent = os.path.dirname(path)

        with open(path) as file:
            yaml_data = yaml.load(file)

            gpickle_file = os.path.join(parent,yaml_data['gpickle_file'])


            self.G = nx.read_gpickle(gpickle_file)
            self.center = yaml_data['center']
            self.scale = yaml_data['scale']
    
    def angle_between(self,p1, p2):
        """
        Calculates the angle between 2 points
        @param p1 tuple (x,y)
        @param p2 tuple (x,y)
        @return b float
        """
        b = np.arctan2(p2[1]-p1[1],p2[0]-p1[0])
        return b

    def getPathPoints(self,nodes):
        """
        Generates the list of points from one node to another
        @param nodes list of nodes
        @return list of points
        """
        for node in nodes:
            if node not in self.G:
                return None

        if(len(nodes) == 1):
            return [[self.G.nodes[nodes[-1]]['x'],self.G.nodes[nodes[-1]]['y']]]

        pointsPath = np.empty([0,2])
        for i in range(len(nodes)-1):
            node1 = nodes[i]
            node2 = nodes[i+1]
            points = self.G.edges[(node1, node2)]['points']
            # Check starting point
            if(np.all(((self.G.nodes[node1]['x'],self.G.nodes[node1]['y']) != points[0]))):
                points = points[::-1]
            pointsPath = np.append(pointsPath,points[1:],axis=0)

        uniq, index = np.unique(pointsPath,axis=0, return_index=True)
        # return uniq[index.argsort()]
        return pointsPath
    
    def getGoalArray(self,points):
        """
        Adds heading to the points
        @param points list of points
        @return list of goals
        """
        GoalArray = []
        heading = 0
        if(len(points) == 1):
            GoalArray.append([points[0][0],points[0][1],heading])
            return GoalArray

        for i in range(len(points)-1):
            p1 = points[i]
            p2 = points[i+1]
            heading = self.angle_between(p1, p2)
            GoalArray.append([p1[0],p1[1],heading])
        GoalArray.append([p2[0],p2[1],heading])
        return GoalArray[::-1]
    
    def get_quaternion_from_euler(self,roll, pitch, yaw):
        """
        Converts heading from euler to quaternion
        @param roll float
        @param pitch float
        @param yaw float
        @return quaternion as a list in the form  (x, y, z, w)
        """
     
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]
    
    def convertToRosMsg(self,goals):
        """
        Converts goals to ros message
        @param goals list of goals
        @return PoseArray
        """
        msg = PoseArray()
        msg.header = Header()
        msg.header.frame_id = "odom"
        pose_list = []

        for goal in goals:
            pose_msg = Pose()
            pose_msg.position.x = goal[0]
            pose_msg.position.y = goal[1]
            pose_msg.position.z = 0
            q = self.get_quaternion_from_euler(0,0,goal[2])
            pose_msg.orientation.x = q[0]
            pose_msg.orientation.y = q[1]
            pose_msg.orientation.z = q[2]
            pose_msg.orientation.w = q[3]
        
            pose_list.append(pose_msg)
        
        msg.poses = pose_list
        return msg

        
    def getGoals(self,nodes):
        """
        Converts list of nodes to goals in ros message format
        @param nodes list of nodes
        @return PoseArray
        """
        try:
            points = self.getPathPoints(nodes)
            goals = self.getGoalArray(points)
            return self.convertToRosMsg(goals)
        except:
            return None
    
    def convertToLngLat(self, point):
        """
        Converts xy point to GPS coordinate
        @param point tuple (x,y)
        @return gps coordinate tuple (lon,lat)
        """
        
        x = point[0]*(1/self.scale)
        y = point[1]*(1/self.scale)

        R = 6371009
        brng = np.arctan2(x,y)
        d = np.linalg.norm((x,y))

        lat1 = math.radians(self.center['lat']) #Current lat point converted to radians
        lon1 = math.radians(self.center['lon'])

        lat2 = math.asin( math.sin(lat1)*math.cos(d/R) +
        math.cos(lat1)*math.sin(d/R)*math.cos(brng))

        lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1),
                    math.cos(d/R)-math.sin(lat1)*math.sin(lat2))

        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)

        return(lon2,lat2)
