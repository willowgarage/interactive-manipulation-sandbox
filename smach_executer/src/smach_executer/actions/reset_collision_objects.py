'''
Action for resetting attached/unattached collision objects and the collision map
'''
import rospy
import actionlib
from smach import State

from pr2_object_manipulation_msgs.msg import IMGUIAction, IMGUIGoal
from actionlib_msgs.msg import GoalStatus

class ResetCollisionObjects(State):
    """
    Calls the Interactive Manipulation backend's IMGUIAction for resetting collision objects and the collision map

    input keys:
    map is boolean, true or false (do you want the collision map reset?)
    unattached_objects is boolean, true or false (do you want unattached objects (not in-hand) reset?)
    attached_objects is boolean, true or false (do you want attached objects reset?)  
    arm is 'right', 'left', or 'both' (applies to attached objects only: reset the object held by which arm(s)?)

    return values:
    * 'succeeded'  In this case, the goal(s) have been sent to the IMGUIAction.
    * 'failed'  Some error occurred.
    """

    def __init__(self, input_keys=['map', 'unattached_objects', 'attached_objects', 'arm']):
        action_uri = 'imgui_action'
        self.action_client = actionlib.SimpleActionClient(action_uri, IMGUIAction)
        rospy.loginfo("waiting for %s"%action_uri)
        self.action_client.wait_for_server()
        rospy.loginfo("%s found"%action_uri)
        State.__init__(self, outcomes=['succeeded', 'failed'], input_keys = input_keys)
                       
    def execute(self, userdata):
        goal = IMGUIGoal()

        goal.command.command = goal.command.RESET

        #reset everything
        if userdata['attached_objects'] and \
                userdata['arm'] == 'both' and \
                userdata['unattached_objects'] and \
                userdata['map']:
            goal.options.reset_choice = 0
            rospy.loginfo("Sending IMGUI goal to reset map and all collision objects")
            self.action_client.send_goal(goal)
            return 'succeeded'

        #reset attached objects
        if userdata['attached_objects']:
            if userdata['arm'] == 'left':
                goal.options.arm_selection = 1
                goal.options.reset_choice = 3
            elif userdata['arm'] == 'right':
                goal.options.arm_selection = 0
                goal.options.reset_choice = 3
            elif userdata['arm'] == 'both':
                goal.options.reset_choice = 2
            rospy.loginfo("Sending IMGUI goal to reset attached objects for arm(s): {0}".format(userdata['arm']))
            self.action_client.send_goal(goal)

        #reset unattached objects
        if userdata['unattached_objects']:
            goal.options.reset_choice = 1
            rospy.loginfo("Sending IMGUI goal to reset unattached collision objects")
            self.action_client.send_goal(goal)            

        #reset collision map
        if userdata['map']:
            goal.options.reset_choice = 4
            rospy.loginfo("Sending IMGUI goal to reset collision map")
            self.action_client.send_goal(goal)

        rospy.loginfo("sending IMGUI goal(s) succeeded")
        return 'succeeded'
        
