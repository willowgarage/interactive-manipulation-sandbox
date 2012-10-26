'''
Action for pointing the head.
'''
import rospy
import actionlib
from smach import State


from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

class PointHead(State):
    """
    Points the head at a position in space.

    target_x, target_y, and target_z are the coordinates of the point, in base_link frame.
    
    * 'succeeded'  In this case, the robot is guaranteed to be at the goal pose.
    * 'failed' Robot not necessarily at the goal pose.
    """

    HEAD_TIMEOUT_DEFAULT = 3.0  # default timeout in seconds

    def __init__(self, input_keys=['target_x', 'target_y', 'target_z', 'target_frame',
            'pointing_frame', 'pointing_x', 'pointing_y', 'pointing_z']):
        action_uri = '/head_traj_controller/point_head_action'
        self.point_head_client = actionlib.SimpleActionClient(action_uri, PointHeadAction)
        rospy.loginfo("waiting for %s"%action_uri)
        self.point_head_client.wait_for_server()
        rospy.loginfo("%s found"%action_uri)
        State.__init__(self, outcomes=['succeeded', 'failed'], input_keys = input_keys)
        
        self.head_timeout = rospy.get_param('~head_timeout', PointHead.HEAD_TIMEOUT_DEFAULT)
                       
    def execute(self, userdata):
        goal = PointHeadGoal()
        goal.target.point.x = userdata['target_x']
        goal.target.point.y = userdata['target_y']
        goal.target.point.z = userdata['target_z']
        goal.target.header.stamp = rospy.Time.now()
        if not userdata['target_frame']:
            goal.target.header.frame_id = 'base_link'
        else:
            goal.target.header.frame_id = userdata['target_frame']

        goal.pointing_frame = userdata['pointing_frame']
        goal.pointing_axis.x = userdata['pointing_x']
        goal.pointing_axis.y = userdata['pointing_y']
        goal.pointing_axis.z = userdata['pointing_z']
        rospy.loginfo("Sending head pointing goal (%f, %f, %f) and waiting for result" % (
                goal.target.point.x, goal.target.point.y, goal.target.point.z))

        # send the goal
        self.point_head_client.send_goal(goal)
        finished_within_time = self.point_head_client.wait_for_result(rospy.Duration(self.head_timeout))
        if not finished_within_time:
            self.point_head_client.cancel_goal()
            return 'failed'
        return 'succeeded'
