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

class NavigateToPose(State):
    """
    Navigates to a 2d pose.  Possible outcomes:
    
    * 'succeeded'  In this case, the robot is guaranteed to be at the goal pose.
    * 'failed' Robot not necessarily at the goal pose.
    * 'preempted' Goal was preempted.
    """
    
    def __init__(self, input_keys=['frame_id', 'x', 'y', 'theta']):
        move_base_uri = '/move_base'
        self.move_base_client = actionlib.SimpleActionClient(move_base_uri, mbm.MoveBaseAction)
        rospy.loginfo("waiting for move base server")
        self.move_base_client.wait_for_server()
        rospy.loginfo("move base server found")
        State.__init__(self, outcomes = ['succeeded', 'failed', 'preempted'], input_keys = input_keys)

    def execute(self, userdata):
        frame_id = userdata['frame_id']
        x = userdata['x']
        y = userdata['y']
        theta = userdata['theta']
        
        goal = mbm.MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.pose = _to_pose(x, y, theta)

        rospy.loginfo("Sending base goal (%f, %f, %f) and waiting for result" % (x, y, theta))
        self.move_base_client.send_goal(goal)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.move_base_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            state = self.move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("navigation succeeded")
                return 'succeeded'
            elif state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                rospy.loginfo("state was:"+str(state))
                return 'failed'
            r.sleep()

    def request_preempt(self):
         State.request_preempt(self)
         rospy.loginfo("NavigateToPose was preempted")


def _yaw(q):
    e = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return e[2]

def _to_quaternion(yaw):
    return gm.Quaternion(*trans.quaternion_from_euler(0, 0, yaw))

def _to_pose(x, y, theta):
    return gm.Pose(gm.Point(x, y, 0), _to_quaternion(theta))
