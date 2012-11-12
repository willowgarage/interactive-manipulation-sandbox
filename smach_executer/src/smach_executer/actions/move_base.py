'''
Action for moving the base. Mostly copied from sushi/pr2_python/base.py
'''
import rospy
import actionlib
import tf.transformations as trans
import geometry_msgs.msg as gm
import move_base_msgs.msg as mbm
from smach import State
from actionlib_msgs.msg import GoalStatus

class MoveBase(State):
    """
    Move the base a small amount.

    Parameters: x, y, theta -- amount to translate base relative to robot coordinates
    
    Possible outcomes:
    
    * 'succeeded'  In this case, the robot is guaranteed to be at the goal pose.
    * 'failed' Robot not necessarily at the goal pose.
    * 'preempted' Goal was preempted.
    """
    
    def __init__(self, input_keys=['x', 'y', 'theta']):
        move_base_uri = '/base_controller/command'
        self.move_base_client = actionlib.SimpleActionClient(move_base_uri, mbm.MoveBaseAction)
        rospy.loginfo("waiting for /base_controller/command server")
        self.move_base_client.wait_for_server(timeout=rospy.Duration(10))
        rospy.loginfo("/base_controller/command server found")
        State.__init__(self, outcomes = ['succeeded', 'failed', 'preempted'], input_keys = input_keys)

    def execute(self, userdata):
        x = userdata['x']
        y = userdata['y']
        theta = userdata['theta']

        goal = gm.Twist()
        goal.linear.x = x
        goal.linear.y = y
        goal.angular.z = theta
        
        rospy.loginfo("Sending move goal (%f, %f, %f) and waiting for result" % (x, y, theta))
        self.move_base_client.send_goal(goal)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.move_base_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            state = self.move_base_client.get_state()
            rospy.loginfo("Move base has state: %s" % state)
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("move succeeded")
                return 'succeeded'
            elif state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                rospy.loginfo("state was: "+str(state))
                return 'failed'
            r.sleep()

    def request_preempt(self):
         State.request_preempt(self)
         rospy.loginfo("MoveBase was preempted")

