import json
import rospy
import actionlib

# used to convert between json strings and ROS messages
from rosbridge_library.internal import message_conversion

from activity_msgs.msg import StartActivityAction, StartActivityResult, StopActivityAction, StopActivityResult, ActivityErrorCode
from activity_manager.threaded_activity_manager import ThreadedActivityManager
from activity_manager.activity_loader import ActivityLoader

class ActivityServer:
    def __init__(self, base_topic):
        self._activity_manager = ThreadedActivityManager()
        self._activity_loader = ActivityLoader()

        start_activity_topic = '%s/start_activity' % base_topic
        self._start_activity_as = actionlib.SimpleActionServer(
            start_activity_topic, StartActivityAction, self.start_activity_cb, False)

        stop_activity_topic = '%s/stop_activity' % base_topic
        self._stop_activity_as = actionlib.SimpleActionServer(
            stop_activity_topic, StopActivityAction, self.stop_activity_cb, False)

        self._start_activity_as.start()
        self._stop_activity_as.start()

    def start_activity_cb(self, goal):
        rospy.loginfo('Starting activity: %s' % goal.activity_type)

        # find the python class that implements this activity
        activity_class = self._activity_loader.get_activity_class(goal.activity_type)
        activity_goal_class = self._activity_loader.get_activity_goal_class(goal.activity_type)

        # convert the goal from JSON to a ROS message
        activity_goal_dict = json.loads(goal.goal)
        activity_goal = message_conversion.populate_instance(activity_goal_dict, activity_goal_class)

        # ask the activity manager to start the activity
        activity_id = self._activity_manager.start_activity(activity_class, activity_goal, goal.desired_activity_id)

        # started successfully (even though the activity itself may end up failing)
        result = StartActivityResult()
        result.activity_id = activity_id
        result.error_code.val = ActivityErrorCode.SUCCESS
        self._start_activity_as.set_succeeded(result)
        return

    def stop_activity_cb(self, goal):
        rospy.loginfo('Stopping activity')

        # ask the activity manager to stop the activity
        self._activity_manager.stop_activity(goal.activity_id)
        
        result = StopActivityResult()
        result.error_code.val = ActivityErrorCode.SUCCESS        
        self._stop_activity_as.set_succeeded(result)
        return

        
    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
