# !/usr/local/bin/python
import rospy
from rospy.exceptions import ROSTimeMovedBackwardsException

class StateMachine:
    
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []

    def add_state(self, name, handler, end_state=0):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name.upper()

    def run(self, ):
        try:
            handler = self.handlers[self.startState]
        except:
            rospy.logwarn("must call .set_start() before .run()")
        if not self.endStates:
            rospy.logwarn("at least one state must be an end_state")

        old_state = ""
        while not rospy.is_shutdown():
            try:
                (newState) = handler()
                if(newState != old_state):
                    rospy.loginfo(newState)
                    old_state = newState    
                if newState.upper() in self.endStates:
                    rospy.loginfo("reached ", newState)
                    break 
                else:
                    handler = self.handlers[newState.upper()]    
            except KeyboardInterrupt:
                break