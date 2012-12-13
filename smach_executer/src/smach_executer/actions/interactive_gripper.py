'''
Action for creating the gripper interactive marker for grasping, placing, or moving the arm
'''
import rospy
import actionlib
from smach import State

from pr2_object_manipulation_msgs.msg import IMGUIAction, IMGUIGoal
from actionlib_msgs.msg import GoalStatus

class InteractiveGripper(State):
    """
    Calls the Interactive Manipulation backend's IMGUIAction for grasping, placing, or arm movement

    action is 'grasp', 'place', or 'move'
    arm is 'right' or 'left'
    lift is boolean, true or false (do you want the grasp to execute a lift/place to retreat the gripper afterwards?)

    * 'succeeded'  In this case, the goal has been sent to the IMGUIAction.
    * 'failed'  Some error occurred.
    """

    def __init__(self, input_keys=['arm', 'action', 'lift']):
        action_uri = 'imgui_action'
        self.action_client = actionlib.SimpleActionClient(action_uri, IMGUIAction)
        rospy.loginfo("waiting for %s"%action_uri)
        self.action_client.wait_for_server()
        rospy.loginfo("%s found"%action_uri)
        State.__init__(self, outcomes=['succeeded', 'failed'], input_keys = input_keys)
                       
    def execute(self, userdata):
        goal = IMGUIGoal()

        if userdata['action'] == 'grasp':
            goal.command.command = goal.command.PICKUP
        elif userdata['action'] == 'place':
            goal.command.command = goal.command.PLACE
        elif userdata['action'] == 'move':
            goal.command.command = goal.command.PLANNED_MOVE
        else:
            rospy.logerr("action %s not recognized!"%userdata['action'])
            return 'failed'

        if userdata['arm'] == 'left':
            goal.options.arm_selection = 1
        else:
            goal.options.arm_selection = 0

        goal.options.collision_checked = 1
        if userdata['lift'] == True:
            goal.options.adv_options.lift_steps = 10
            goal.options.adv_options.retreat_steps = 10
        else:
            goal.options.adv_options.lift_steps = 0
            goal.options.adv_options.retreat_steps = 0
        goal.options.adv_options.find_alternatives = True
        goal.options.adv_options.desired_approach = 10
        goal.options.adv_options.min_approach = 5
        goal.options.adv_options.max_contact_force = 50

        rospy.loginfo("Sending IMGUI goal for %s arm to execute a %s"%(userdata['arm'], userdata['action']))

        # send the goal
        self.action_client.send_goal(goal)
        rospy.loginfo("sending IMGUI goal succeeded")
        return 'succeeded'
        
