'''
Action for moving the torso.
'''
import rospy
import actionlib
from smach import State

from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from actionlib_msgs.msg import GoalStatus

class MoveTorso(State):
    """
    Moves the PR2 torso to a specified position.

    position: the torso position.  0 is down, 0.295 is up.
    
    * 'succeeded'  Torso made it to the desired position.
    * 'preempted'  Someone preempted the action.
    * 'failed'  Torso didn't make it / some error occurred.
    """
    
    TORSO_TIMEOUT_DEFAULT = 30. #default timeout in seconds

    def __init__(self, input_keys=['position']):
        action_uri = '/torso_controller/position_joint_action'
        self.torso_client = actionlib.SimpleActionClient(action_uri, SingleJointPositionAction)
        rospy.loginfo("waiting for %s"%action_uri)
        self.torso_client.wait_for_server()
        rospy.loginfo("%s found"%action_uri)
        State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'], input_keys = input_keys)
        
        self.torso_timeout = rospy.get_param('~torso_timeout', MoveTorso.TORSO_TIMEOUT_DEFAULT)
                       
    def execute(self, userdata):
        goal = SingleJointPositionGoal()
        goal.position = userdata.position
        rospy.loginfo("Sending torso goal with position %0.3f and waiting for result"%userdata.position) 

        # send the goal
        self.torso_client.send_goal(goal)
        start_time = rospy.get_rostime()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            if now - start_time > rospy.Duration(self.torso_timeout):
                rospy.loginfo("torso timed out!")
                self.torso_client.cancel_goal()
                return 'failed'
            if self.preempt_requested():
                rospy.loginfo("torso goal preempted!")
                self.torso_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            state = self.torso_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                break
            r.sleep()
        rospy.loginfo("move torso succeeded")
        return 'succeeded'
