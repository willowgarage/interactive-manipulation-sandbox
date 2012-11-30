import traceback
import json
import rospy
import actionlib
import smach_ros

from executer_actions.msg import ExecuteAction, ExecuteResult
from smach_executer import parser

# import actions
from smach_executer.actions.dummy import Dummy
from smach_executer.actions.navigate_to_pose import NavigateToPose
from smach_executer.actions.plug_in import PlugIn
from smach_executer.actions.unplug import Unplug
from smach_executer.actions.tuck_arms import TuckArms
from smach_executer.actions.point_head import PointHead
from smach_executer.actions.point_head_in_image import PointHeadInImage
from smach_executer.actions.segment_and_recognize import SegmentAndRecognize
from smach_executer.actions.pickup_object import PickupObject
from smach_executer.actions.move_torso import MoveTorso

class ExecuterServer:
    def __init__(self, debug_mode=False):
        self.debug_mode = debug_mode
        
        self.actionlib_server = actionlib.SimpleActionServer(
            'executer/execute', ExecuteAction, self.execute, False)
        self.actionlib_server.register_preempt_callback(self.preempt)

        # set of actions should eventually be found dynamically, instead of defined explicitly here
        self.actions = {
            'Dummy': Dummy,
            'NavigateToPose': NavigateToPose,
            'PlugIn': PlugIn,
            'Unplug': Unplug,
            'TuckArms': TuckArms,
            'PointHead': PointHead,
            'PointHeadInImage': PointHeadInImage,
            'SegmentAndRecognize': SegmentAndRecognize,
            'PickupObject': PickupObject,
            'MoveTorso': MoveTorso
            }

        self.sm = None
        
    def start(self):
        rospy.loginfo('Starting Executer server')
        self.actionlib_server.start()

    def preempt(self):
         rospy.loginfo("Executer Server: got preempt")
         if self.sm:
             self.sm.request_preempt()
         result = ExecuteResult()
         result.retval = result.RETVAL_SUCCESS
         result.outcome = 'preempted'
         self.actionlib_server.set_preempted(result)

    def execute(self, goal):
        rospy.loginfo('Got goal:\n %s' % str(goal))

        # parse the json string
        try:
            action_dict = json.loads(goal.action)
        except KeyError as e:
            rospy.logerr('Unable to parse json string (%s)' % str(e))
            result = ExecuteResult()
            result.retval = result.RETVAL_PARSE_ERROR
            result.error_string = str(e)
            self.actionlib_server.set_succeeded(result)
            return

        # execute the action
        try:
            (outcome, outputs) = self.execute_action(action_dict)
        except ValueError as e:
            rospy.logerr('Runtime error while executing state machine: %s' % str(e))
            traceback.print_exc()
            result = ExecuteResult()
            result.retval = result.RETVAL_RUNTIME_ERROR
            result.error_string = str(e)
            self.actionlib_server.set_succeeded(result)
            return

        # set_preempted is done in the preempted callback function
        if self.actionlib_server.is_preempt_requested():
            return

        # finished successfully (even though the action itself may have failed)
        result = ExecuteResult()
        result.retval = result.RETVAL_SUCCESS
        try:
            result.outputs = json.dumps(outputs)
        except Exception as e:
            rospy.logerr("Unable to dump outputs to json string: %s"%str(e))
        result.outcome = outcome
        self.actionlib_server.set_succeeded(result)
        return
            
    def execute_action(self, action_dict):
        # create a state machine wrapper for the action
        self.sm = parser.create_state_machine_from_action_dict(action_dict, self.actions)
        
        # execute the state machine
        rospy.loginfo('Executing state machine')
        sis = smach_ros.IntrospectionServer('executer_server_introspection', self.sm, '/EXECUTER_SERVER')
        sis.start()
        outcome = self.sm.execute()
        outputs = ""
        registered_output_keys = self.sm.get_registered_output_keys()
        rospy.loginfo("registered output keys: %s"%str(registered_output_keys))
        if "outputs" in self.sm.get_registered_output_keys():
            outputs = self.sm.userdata.outputs
        else:
            rospy.loginfo("no outputs in state's registered output keys")

        # state machine executed successfully; return outcome
        rospy.loginfo('State machine executed successfully with outcome: %s' % outcome)
        return (outcome, outputs)
