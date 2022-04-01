# !/usr/local/bin/python
import rospy

class StateMachine:
    """StateMachine
    This module is a template for state machine
    It can add, states and handles the running
    """
    
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []

    def add_state(self, name, handler, end_state=0):
        """
        Add a state to the state machine
        @param name string
        @handler function
        """
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        """
        set the start state of the state machine
        @param name string
        """
        self.startState = name.upper()

    def run(self, ):
        """
        Run the state machine
        """
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