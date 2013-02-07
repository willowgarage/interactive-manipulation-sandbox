'''
Action for moving the arm to a set of joint angles (collision-aware, with motion planning)
'''
import rospy
import actionlib
from smach import State

from actionlib_msgs.msg import GoalStatus
from arm_navigation_msgs.msg import MoveArmAction, MoveArmGoal, JointConstraint
from arm_navigation_msgs.srv import SetPlanningSceneDiff, SetPlanningSceneDiffRequest

class MoveArmToJoint(State):
    """
    Moves the arm to a set of joint goals (collision-aware, with motion planning)

    arm_angles is the set of joint angles for the arm
    arm is 'right' for the right arm, and 'left' for the left arm

    * 'succeeded'  In this case, the robot is guaranteed to be within tolerance of the goal angles
    * 'failed' Robot not necessarily at the goal angles.
    """

    ARM_TIMEOUT_DEFAULT = 30.0  # default timeout in seconds

    def __init__(self, input_keys=['arm_angles', 'arm']):
        
        self.arm_timeout = rospy.get_param('~arm_timeout', MoveArmToJoint.ARM_TIMEOUT_DEFAULT)

        #create the action clients
        self.move_arm_clients = {}
        self.move_arm_clients['right'] = actionlib.SimpleActionClient('move_right_arm', MoveArmAction)
        self.move_arm_clients['left'] = actionlib.SimpleActionClient('move_left_arm', MoveArmAction)
        rospy.loginfo("waiting for move arm servers")
        self.move_arm_clients['right'].wait_for_server()
        self.move_arm_clients['left'].wait_for_server()
        rospy.loginfo("move arm servers were there")

        rospy.loginfo("waiting for set planning scene diff service")
        rospy.wait_for_service("environment_server/set_planning_scene_diff")
        self.set_planning_scene_srv = rospy.ServiceProxy("environment_server/set_planning_scene_diff", SetPlanningSceneDiff)

        #joint names for the arm
        joint_names = ["_shoulder_pan_joint", 
                       "_shoulder_lift_joint", 
                       "_upper_arm_roll_joint", 
                       "_elbow_flex_joint", 
                       "_forearm_roll_joint", 
                       "_wrist_flex_joint", 
                       "_wrist_roll_joint"]
        self.joint_names = {}
        self.joint_names['right'] = ['r' + x for x in joint_names]
        self.joint_names['left'] = ['l' + x for x in joint_names]

        State.__init__(self, outcomes=['succeeded', 'failed'], input_keys = input_keys)


    ##set the planning scene (so IK and move_arm can run)
    def set_planning_scene(self, ordered_collision_ops = None, link_padding = None):
        req = SetPlanningSceneDiffRequest()
        if ordered_collision_ops != None:
            req.operations = ordered_collision_ops
        if link_padding != None:
            req.planning_scene_diff.link_padding = link_padding
        try:
            resp = self.set_planning_scene_srv(req)
        except:
            rospy.logerr("error in calling set_planning_scene_diff!")
            return 0
        return 1


    ##create a basic goal message for move_arm
    def create_move_arm_goal(self, arm):
        goal = MoveArmGoal()
        goal.motion_plan_request.group_name = arm + "_arm"
        goal.motion_plan_request.num_planning_attempts = 2;
        goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0);
        goal.motion_plan_request.planner_id = ""
        goal.planner_service_name = "ompl_planning/plan_kinematic_path"
        return goal


    ##send a goal to move arm and wait for the result
    #returns 0 if timed out; otherwise an ArmNavigationErrorCodes value (int32, 1=success) from move_arm 
    def send_move_arm_goal(self, arm, goal):

        #send the goal off
        self.move_arm_clients[arm].send_goal(goal)

        #wait for the move to finish and return the result
        finished_within_time = self.move_arm_clients[arm].wait_for_result(rospy.Duration(60))
        if not finished_within_time:
            self.move_arm_clients[arm].cancel_goal()
            rospy.logerr("Timed out when asking move_arm to go to a joint goal")
            return 0

        state = self.move_arm_clients[arm].get_state()
        result = self.move_arm_clients[arm].get_result()
        if state == GoalStatus.SUCCEEDED and result.error_code.val == 1:
            rospy.loginfo("move_arm succeeded")
        else:
            rospy.logerr("move_arm did not succeed, state %d, error code %d"%(state, result.error_code.val))
        return result.error_code.val


    ##pretty-print list to string
    def pplist(self, list):
        return ' '.join(['%5.3f'%x for x in list])


    ##use move_arm to get to a desired joint configuration in a collision-free way
    #returns 0 if timed out; otherwise an ArmNavigationErrorCodes (int32, 1=success) from move_arm 
    def move_arm_joint(self, arm, joint_angles):

        self.set_planning_scene()

        rospy.loginfo("asking move_arm to send the %s arm to go to angles: %s"%(arm, self.pplist(joint_angles)))

        goal = self.create_move_arm_goal(arm)

        #add the joint angles as a joint constraint
        for (joint_name, joint_angle) in zip(self.joint_names[arm], joint_angles):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_angle
            joint_constraint.tolerance_below = .1
            joint_constraint.tolerance_above = .1
            goal.motion_plan_request.goal_constraints.joint_constraints.append(joint_constraint)

        #send the goal off to move arm
        result = self.send_move_arm_goal(arm, goal)
        return result


    def execute(self, userdata):
        
        result = self.move_arm_joint(userdata['arm'], userdata['arm_angles'])
        if result == 1:
            return 'succeeded'
        return 'failed'
