'''
Action for picking up a segmented or recognized object
'''
import rospy
import actionlib
from smach import State

from pr2_object_manipulation_msgs.msg import PickupIMObjectAction, PickupIMObjectGoal
from actionlib_msgs.msg import GoalStatus

class PickupObject(State):
    """
    Picks up an already-segmented or recognized object.

    arm is 'right' or 'left'.
    object_id is the id number for the object (as given in SegmentAndRecognize)

    * 'succeeded'  In this case, the goal has been sent to the Pickup action.
    * 'failed'  Some error occurred (like the object not existing).
    """


    def __init__(self, input_keys=['arm', 'object_id']):
        action_uri = 'pickup_im_object_action'
        self.pickup_client = actionlib.SimpleActionClient(action_uri, PickupIMObjectAction)
        rospy.loginfo("waiting for %s"%action_uri)
        self.pickup_client.wait_for_server()
        rospy.loginfo("%s found"%action_uri)
        State.__init__(self, outcomes=['succeeded', 'failed'], input_keys = input_keys)
                       
    def execute(self, userdata):
        goal = PickupIMObjectGoal()
        if userdata['arm'] == 'left':
            goal.arm_selection = 1
        else:
            goal.arm_selection = 0
        goal.object_id = userdata.object_id
        rospy.loginfo("Sending pickup object goal for %s arm to pick up object %d"%(userdata['arm'], goal.object_id))

        # send the goal
        self.pickup_client.send_goal(goal)
        finished_within_time = self.pickup_client.wait_for_result(rospy.Duration(5.0))
        if not finished_within_time:
             self.pickup_client.cancel_goal()
             return 'failed'
        state = self.pickup_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("sending pickup succeeded")
            return 'succeeded'
        else:
            print state
            rospy.loginfo("pickup returned error")
            return 'failed'
