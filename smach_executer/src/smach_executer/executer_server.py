import json
import rospy
import actionlib

from smach_executer.msg import ExecuterInterfaceAction, ExecuterInterfaceResult
from smach_executer import parser
from smach_executer.actions.dummy import Dummy
from smach_executer.actions.navigate_to_pose import NavigateToPose

class ExecuterServer:
    def __init__(self, debug_mode=False):
        self.debug_mode = debug_mode
        
        self.actionlib_server = actionlib.SimpleActionServer(
            'executer_interface', ExecuterInterfaceAction, self.execute, False)

        # set of actions should eventually be found dynamically, instead of defined explicitly here
        self.actions = {'Dummy': Dummy, 'NavigateToPose': NavigateToPose}

    def start(self):
        rospy.loginfo('Starting Executer server')
        self.actionlib_server.start()

    def execute(self, goal):
        rospy.logdebug('Got goal:\n %s' % str(goal))

        # parse the json string
        try:
            request_dict = json.loads(goal.json_str)
        except KeyError as e:
            rospy.logerr('Unable to parse json string (%s)' % (str(e),))
            result = ExecuterInterfaceResult()
            result.retval = result.RETVAL_PARSE_ERROR
            result.error_string = str(e)
            self.actionlib_server.set_succeeded(result)
            return

        # check what operation is being requested
        try:
            op = request_dict['op']
        except ValueError as e:
            rospy.logerr('JSON message missing "op" field')
            result = ExecuterInterfaceResult()
            result.retval = result.RETVAL_PARSE_ERROR
            result.error_string = str(e)
            self.actionlib_server.set_succeeded(result)
            return

        if op == 'execute':
            try:
                action_dict = request_dict['action']
            except KeyError as e:
                rospy.logerr('Execute requested but no "action" field specified')
                result = ExecuterInterfaceResult()
                result.retval = result.RETVAL_PARSE_ERROR
                result.error_string = str(e)
                self.actionlib_server.set_succeeded(result)
                return

            # execute the operation
            try:
                outcome = self.execute_action(action_dict)
            except ValueError as e:
                rospy.logerr('Runtime error while executing state machine: %s' % str(e))
                result = ExecuterInterfaceResult()
                result.retval = result.RETVAL_RUNTIME_ERROR
                result.error_string = str(e)
                result.retval = result.RETVAL_SUCCESS
                self.actionlib_server.set_succeeded(result)
                return
            
            result = ExecuterInterfaceResult()
            result.retval = result.RETVAL_SUCCESS
            result.outcome = outcome
            self.actionlib_server.set_succeeded(result)
            return
        else:
            rospy.logerr('Unkown operation "%s"' % op)
            result = ExecuterInterfaceResult()    
            result.retval = result.RETVAL_PARSE_ERROR
            result.error_string = str(e)
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
