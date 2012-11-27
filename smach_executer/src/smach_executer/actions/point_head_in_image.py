'''
Action for pointing the head.
'''
import rospy
import actionlib
from smach import State

from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point

class PointHeadInImage(State):
    """
    Points the head in the direction of a point in an image (from a head camera).

    target_x, target_y are the normalized ([0,1]) coordinates of the clicked point.
      (target_x,target_y)=(0,0) is image top left, (1,1) is bottom right
    camera_info_topic is the name of the CameraInfo topic to look at for the camera image.

    * 'succeeded'  In this case, the robot is guaranteed to be at the goal pose.
    * 'failed' Robot not necessarily at the goal pose.
    """

    HEAD_TIMEOUT_DEFAULT = 3.0  # default timeout in seconds

    def __init__(self, input_keys=['target_x', 'target_y', 'camera_info_topic']):
        action_uri = '/head_traj_controller/point_head_action'
        self.point_head_client = actionlib.SimpleActionClient(action_uri, PointHeadAction)
        rospy.loginfo("waiting for %s"%action_uri)
        self.point_head_client.wait_for_server()
        rospy.loginfo("%s found"%action_uri)
        State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'], input_keys = input_keys)
        
        self.head_timeout = rospy.get_param('~head_timeout', PointHeadInImage.HEAD_TIMEOUT_DEFAULT)

    # Get a CameraInfo message and use it to convert from normalized image coords to a 3D target point
    def get_target_point(self, x, y, camera_info):
        fx = camera_info.P[0*4+0]
        fy = camera_info.P[1*4+1]
        cx = camera_info.P[0*4+2]
        cy = camera_info.P[1*4+2]
        u = x * camera_info.width
        v = y * camera_info.height
        point = Point()
        point.z = 1.0
        point.x = (u - cx) * point.z / fx
        point.y = (v - cy) * point.z / fy
        return point
                       
    def execute(self, userdata):
        camera_info_topic = userdata['camera_info_topic']
        camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)

        goal = PointHeadGoal()
        x = userdata['target_x']
        y = userdata['target_y']
        goal.target.point = self.get_target_point(x, y, camera_info)
        goal.target.header.frame_id = camera_info.header.frame_id
        goal.target.header.stamp = rospy.Time.now()
        goal.pointing_frame = camera_info.header.frame_id
        goal.pointing_axis.x = 0
        goal.pointing_axis.y = 0
        goal.pointing_axis.z = 1
        rospy.loginfo("Sending head pointing goal (%f, %f, %f) and waiting for result" % (
                goal.target.point.x, goal.target.point.y, goal.target.point.z))

        # send the goal
        self.point_head_client.send_goal(goal)
        start_time = rospy.get_rostime()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            if now - start_time > rospy.Duration(self.head_timeout):
                rospy.loginfo("head timed out!")
                self.point_head_client.cancel_goal()
                return 'failed'
            if self.preempt_requested():
                rospy.loginfo("point head goal preempted!")
                self.point_head_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            state = self.point_head_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                break
            r.sleep()
        rospy.loginfo("point head succeeded")
        return 'succeeded'
