import json
import rospy
import actionlib

from activity_msgs.msg import StartActivityAction, StartActivityResult, StopActivityAction, StopActivityResult, ActivityErrorCode
from activity_msgs.msg import ListRunningActivitiesAction, ListRunningActivitiesResult
from activity_manager.threaded_activity_manager import ThreadedActivityManager

class ActivityServer:
    def __init__(self, base_topic):
        self._activity_manager = ThreadedActivityManager()

        start_activity_topic = '%s/start_activity' % base_topic
        self._start_activity_as = actionlib.SimpleActionServer(
            start_activity_topic, StartActivityAction, self.start_activity_cb, False)

        stop_activity_topic = '%s/stop_activity' % base_topic
        self._stop_activity_as = actionlib.SimpleActionServer(
            stop_activity_topic, StopActivityAction, self.stop_activity_cb, False)

        list_activities_topic = '%s/list_running_activities' % base_topic
        self._list_activities_as = actionlib.SimpleActionServer(
            list_activities_topic, ListRunningActivitiesAction, self.list_running_activities_cb, False)

        self._start_activity_as.start()
        self._stop_activity_as.start()
        self._list_activities_as.start()

    def start_activity_cb(self, goal):
        rospy.loginfo('Starting activity: %s' % goal.activity_type)

        # ask the activity manager to start the activity
        activity_id = self._activity_manager.start_activity(goal.activity_type, goal.goal, goal.desired_activity_id)

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

    def list_running_activities_cb(self, goal):
        rospy.loginfo('Listing running activities')

        result = ListRunningActivitiesResult()
        result.running_activities = self._activity_manager.get_running_activities()
        self._list_activities_as.set_succeeded(result)
        return

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
