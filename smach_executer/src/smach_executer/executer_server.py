import json
import rospy
import actionlib

from executer_actions.msg import ExecuteAction, ExecuteResult
from smach_executer import parser
from smach_executer.actions.dummy import Dummy
from smach_executer.actions.navigate_to_pose import NavigateToPose

class ExecuterServer:
    def __init__(self, debug_mode=False):
        self.debug_mode = debug_mode
        
        self.actionlib_server = actionlib.SimpleActionServer(
            'executer/execute', ExecuteAction, self.execute, False)

        # set of actions should eventually be found dynamically, instead of defined explicitly here
        self.actions = {'Dummy': Dummy, 'NavigateToPose': NavigateToPose}

    def start(self):
        rospy.loginfo('Starting Executer server')
        self.actionlib_server.start()

    def execute(self, goal):
        rospy.logdebug('Got goal:\n %s' % str(goal))

        # parse the json string
        try:
            action_dict = json.loads(goal.action)
        except KeyError as e:
            rospy.logerr('Unable to parse json string (%s)' % (str(e),))
            result = ExecuteResult()
            result.retval = result.RETVAL_PARSE_ERROR
            result.error_string = str(e)
            self.actionlib_server.set_succeeded(result)
            return

        # execute the action
        try:
            outcome = self.execute_action(action_dict)
        except ValueError as e:
            rospy.logerr('Runtime error while executing state machine: %s' % str(e))
            result = ExecuteResult()
            result.retval = result.RETVAL_RUNTIME_ERROR
            result.error_string = str(e)
            result.retval = result.RETVAL_SUCCESS
            self.actionlib_server.set_succeeded(result)
            return

        # finished successfully (even though the action itself may have failed)
        result = ExecuteResult()
        result.retval = result.RETVAL_SUCCESS
        result.outcome = outcome
        self.actionlib_server.set_succeeded(result)
        return
            
    def execute_action(self, action_dict):
        # create a state machine wrapper for the action
        sm = parser.create_state_machine_from_action_dict(action_dict, self.actions)
        
        # execute the state machine
        rospy.loginfo('Executing state machine')
        outcome = sm.execute()

        # state machine executed successfully; return outcome
        rospy.loginfo('State machine executed successfully with outcome: %s' % outcome)
        return outcome
