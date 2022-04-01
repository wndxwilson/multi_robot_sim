#!/usr/bin/env python3
import rospy
import subprocess
import shlex
import rosnode
import json
import rospkg

from multi_robot_sim.srv import MultiRobotSimSpawnRobot, MultiRobotSimSpawnRobotResponse, MultiRobotSimRobot, MultiRobotSimRobotResponse
from multi_robot_sim.msg import MultiRobotSimStatus
from gazebo_msgs.srv import GetModelProperties , DeleteModel

from utils.osm_utils import OsmUtil
from enum import Enum

import datetime
import os 
import pandas as pd

class MovebaseStatus(Enum):
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9

class BackendService():
    def __init__(self):

        # Create Service
        spawn_service = rospy.Service('spawn_robot',MultiRobotSimSpawnRobot,self.handle_spawn_robot)
        del_service = rospy.Service('del_robot',MultiRobotSimRobot,self.handle_del_robot)
        status_service = rospy.Service('robot_status',MultiRobotSimRobot,self.handle_robot_status)

        # Subscriber
        status_sub = rospy.Subscriber("status", MultiRobotSimStatus, self.handle_status)

        # History cache
        self.history = {}
        self.log_data = []

        # osm utils
        path = rospy.get_param("/yaml_path")
        self.gg = OsmUtil(path)

        rospy.on_shutdown(self.handle_shut_down)
    
        rospy.loginfo("backend service started ...")
        rospy.spin()
    
    def handle_shut_down(self):
        if not self.log_data:
            rospy.loginfo("no data found")
            return

        columns_name = ["timestamp","robot_name","status code","status","xy_coordinate","GPS_coordinate"]
        
        rospack = rospkg.RosPack()
        log_dir = os.path.join(rospack.get_path('multi_robot_sim'),'logs')

        try: 
            os.mkdir(log_dir)
        except OSError as error: 
            pass 
        
        date = datetime.datetime.now()
        parent_path  = os.path.join(log_dir,date.strftime("%d-%m-%Y"))

        try: 
            os.mkdir(parent_path)
        except OSError as error: 
            pass 
        
        file_path  = os.path.join(parent_path,date.strftime("%H:%M:%S"))
        
        try: 
            os.mkdir(file_path)
        except OSError as error: 
            pass

        df = pd.DataFrame(self.log_data,columns=columns_name)

        try:
            df.to_csv(os.path.join(file_path,"master_log.csv"))
        except: 
            rospy.logwarn("saving failed") 

        for name in df['robot_name'].unique():
            df_tmp = df[df['robot_name'] == name].drop(columns=['robot_name'])
            try:
                df_tmp.to_csv(os.path.join(file_path,name+"_log.csv"))
            except: 
                rospy.logwarn("saving failed") 

    def handle_spawn_robot(self, req):

        try:
            # Check if robot exist
            rospy.wait_for_service('/gazebo/get_model_properties',5.0)
            gazebo_model_srv = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
            res = gazebo_model_srv(req.robot_name)
            if(res.success):
                return MultiRobotSimSpawnRobotResponse("Robot has already been spawned")
                
            # Spawn robot
            command = "roslaunch multi_robot_sim spawn.launch namespace:={name} x_pos:={x} y_pos:={y} yaw:={yaw}".format(name=req.robot_name,x=req.x,y=req.y,yaw=req.yaw)
            
            args = shlex.split(command)
            subprocess.Popen(args)
            rospy.loginfo("Running " + command)
            return MultiRobotSimSpawnRobotResponse("Spawned")

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return MultiRobotSimSpawnRobotResponse("Failed")

    def handle_del_robot(self, req):


        try:
            # Check if robot exist
            rospy.wait_for_service('/gazebo/get_model_properties',5.0)

            gazebo_model_srv = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
            gazebo_delete_model_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

            res = gazebo_model_srv(req.robot_name)
            if(not res.success):
                return MultiRobotSimRobotResponse("Robot does not exist")

            # Despawn robot
            gazebo_delete_model_srv(req.robot_name)

            # Kill node
            ros_nodes = rosnode.get_node_names(req.robot_name)
            rosnode.kill_nodes(ros_nodes)

            # Remove from history
            if(req.robot_name in self.history):
                del self.history[req.robot_name]

            return MultiRobotSimRobotResponse("Robot deleted")

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return MultiRobotSimRobotResponse("Failed")

    
    def handle_robot_status(self, req):

        if(req.robot_name == ""):
            ret_json = json.dumps(self.history)
            return MultiRobotSimRobotResponse(ret_json)
        
        if(req.robot_name not in self.history):
            return MultiRobotSimRobotResponse("Robot does not exist")

        result = {req.robot_name : self.history.pop(req.robot_name)}
        ret_json = json.dumps(result)
        return MultiRobotSimRobotResponse(ret_json)


    def handle_status(self, msg):
        # Add robot data to history
        lnglat_pos = self.gg.convertToLngLat((msg.position.x,msg.position.y))
        self.history[msg.robot_id] = {  
                                        "status_code" : msg.status,
                                        "status" : MovebaseStatus(msg.status).name,
                                        "position" : (msg.position.x,msg.position.y),
                                        "lnglat_pos" : lnglat_pos
                                     }
                                     
        self.log_data.append([datetime.datetime.now().timestamp(),msg.robot_id,msg.status,MovebaseStatus(msg.status).name,(msg.position.x,msg.position.y),lnglat_pos])

if __name__ == "__main__":
    rospy.init_node('robot_management_service')
    try:
        BackendService()
        rospy.loginfo("shutting down")
    except rospy.ROSInterruptException: 
        pass