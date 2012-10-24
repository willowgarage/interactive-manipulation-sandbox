'''
Action for moving the base. Mostly copied from sushi/pr2_python/base.py
'''
import rospy
from smach_ros import SimpleActionState

from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

class PointHead(SimpleActionState):
    """
    Points the head at a position in space.

    target_x, target_y, and target_z are the coordinates of the point, in base_link frame.
    
    * 'succeeded'  In this case, the robot is guaranteed to be at the goal pose.
    * 'failed' Robot not necessarily at the goal pose.
    """
    def __init__(self, input_keys=['target_x', 'target_y', 'target_z', 'target_frame', 'pointing_frame']):
        action_uri = '/head_traj_controller/point_head_action'
        SimpleActionState.__init__(self, action_uri, PointHeadAction, goal_cb=self.goal_cb, input_keys=input_keys)

    def goal_cb(self, userdata, goal):
        goal = PointHeadGoal()
        goal.target.point.x = userdata['target_x']
        goal.target.point.y = userdata['target_y']
        goal.target.point.z = userdata['target_z']
        goal.target.header.stamp = rospy.Time.now()
        if not userdata['target_frame']:
            goal.target.header.frame_id = 'base_link'
        else:
            goal.target.header.frame_id = userdata['target_frame']
        if not userdata['pointing_frame']:
            goal.pointing_frame = 'head_mount_kinect_rgb_optical_frame'
        else:
            goal.pointing_frame = userdata['pointing_frame']
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        rospy.loginfo("Sending head pointing goal (%f, %f, %f) and waiting for result" % (
                goal.target.point.x, goal.target.point.y, goal.target.point.z))
        return goal

    
