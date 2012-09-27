'''
Action for moving the base. Mostly copied from sushi/pr2_python/base.py
'''
import rospy
import tf.transformations as trans
from smach_ros import SimpleActionState
import geometry_msgs.msg as gm
import move_base_msgs.msg as mbm

class NavigateToPose(SimpleActionState):
    """
    Navigates to a 2d pose.  Possible outcomes:
    
    * 'succeeded'  In this case, the robot is guaranteed to be at the goal pose.
    * 'failed' Robot not necessarily at the goal pose.
    """
    def __init__(self, input_keys=['frame_id', 'x', 'y', 'theta']):
        move_base_uri = '/move_base'
        SimpleActionState.__init__(self, move_base_uri, mbm.MoveBaseAction, goal_cb=self.goal_cb, input_keys=input_keys)

    def goal_cb(self, userdata, goal):
        frame_id = userdata['frame_id']
        x = userdata['x']
        y = userdata['y']
        theta = userdata['theta']
        
        goal = mbm.MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.pose = _to_pose(x, y, theta)
        rospy.loginfo("Sending base goal (%f, %f, %f) and waiting for result" % (x, y, theta))
        return goal

def _yaw(q):
    e = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return e[2]

def _to_quaternion(yaw):
    return gm.Quaternion(*trans.quaternion_from_euler(0, 0, yaw))

def _to_pose(x, y, theta):
    return gm.Pose(gm.Point(x, y, 0), _to_quaternion(theta))
