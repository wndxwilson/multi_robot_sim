#!/usr/bin/env python3

import rospy
import threading
from flask import Flask ,request
from multi_robot_sim.srv import MultiRobotSimSpawnRobot, MultiRobotSimGoalNodes, MultiRobotSimRobot
from utils.osm_utils import OsmUtil

app = Flask(__name__)

threading.Thread(target=lambda: rospy.init_node('Server_node', disable_signals=True)).start()

while(True):
    if(rospy.has_param("/yaml_path")):
        break

path = rospy.get_param("/yaml_path")
gg = OsmUtil(path)

@app.route("/api/robot" , methods=['GET', 'POST','DELETE'])
def spawnRobot():

    if request.method == 'POST':

        robot_name = request.args.get('robot_name', default="robot")
        node = request.args.get('node', type=int)
        yaw = request.args.get('yaw', default=0, type=float)

        # Check for node arg
        if node == None:
            return "Invalid request, node not found"

        # Check if node is valid
        coor = gg.getPathPoints([node])
        if coor == None:
            return "Invalid Node"

        x_pos = coor[0][0]
        y_pos = coor[0][1]
        
        # Spawn robot
        try:    
            rospy.wait_for_service('spawn_robot',5.0)
            spawn_srv = rospy.ServiceProxy('spawn_robot', MultiRobotSimSpawnRobot)
            res = spawn_srv(robot_name,x_pos,y_pos,yaw)
            return res.result

        except (rospy.ServiceException, rospy.ROSException) as e:
            return "Service call failed: %s"%e

    
    if request.method == 'DELETE':
        robot_name = request.args.get('robot_name', default="robot")
        try:    
            rospy.wait_for_service('del_robot',5.0)
            del_srv = rospy.ServiceProxy('del_robot', MultiRobotSimRobot)
            res = del_srv(robot_name)
            return res.result

        except (rospy.ServiceException, rospy.ROSException) as e:
            return "Service call failed: %s"%e

    
    if request.method == 'GET':
        robot_name = request.args.get('robot_name', default=None)
        try:    
            rospy.wait_for_service('robot_status',5.0)
            status_srv = rospy.ServiceProxy('robot_status', MultiRobotSimRobot)
            res = status_srv(robot_name)
            return res.result

        except (rospy.ServiceException, rospy.ROSException) as e:
            return "Service call failed: %s"%e

@app.route("/api/navigation/move" , methods=['POST'])
def move():
    json = request.json
    print(json["robot_name"])
    try: 
        rospy.wait_for_service(json["robot_name"]+'/send_waypoint',5.0)
        move_srv = rospy.ServiceProxy(json["robot_name"]+'/send_waypoint', MultiRobotSimGoalNodes)
        res = move_srv(json["nodes"],json['mode'])

        if(res.result):
            return "robot moving"

        return "failed"
    except (rospy.ServiceException, rospy.ROSException) as e:
        return "Service call failed: %s"%e

@app.route("/api/test")
def test():
    return "API works"

if __name__ == '__main__':
    app.run()