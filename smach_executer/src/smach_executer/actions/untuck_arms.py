'''
Action for untucking the PR2's arms.
'''
import rospy
import tf.transformations as trans
from smach_ros import SimpleActionState
from pr2_common_action_msgs.msg import TuckArmsAction, TuckArmsGoal

class UntuckArms(SimpleActionState):
    """
    Untucks the PR2's arms.
    
    * 'succeeded'  In this case, the robot is guaranteed to be at the goal pose.
    * 'failed' Robot not necessarily at the goal pose.
    """
    def __init__(self, input_keys=[]):
        tuck_arms_uri = '/tuck_arms'
        SimpleActionState.__init__(self, tuck_arms_uri, TuckArmsAction, goal_cb=self.goal_cb, input_keys=input_keys)

    def goal_cb(self, userdata, goal):
        goal = TuckArmsGoal()
        goal.tuck_left = False
        goal.tuck_right = False
        rospy.loginfo("Sending tuck arms goal and waiting for result")
        return goal

