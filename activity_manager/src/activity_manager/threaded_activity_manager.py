import threading
import json
import rospy
from activity_msgs.msg import RunningActivityInfo

# used to convert between json strings and ROS messages
from rosbridge_library.internal import message_conversion

from activity_manager.activity_handle import ActivityHandle
from activity_manager.activity_loader import ActivityLoader

def generate_unique_id(current_ids, desired_id=None):
    '''
    Create a unique activity ID.

    Args:
        current_ids (sequence of int) : All currently existing activity IDs
        desired_id (int): If provided, will be used unless not unique.

    Returns:
        new_id (int)
    '''
    if desired_id is None or desired_id in current_ids:
        # if the desired id number is already taken, pick a new one
        new_id = max(current_ids) + 1
    else:
        new_id = desired_id
    return new_id


class ThreadedActivityManager:
    def __init__(self):
        '''
        Manages running activities. Provides the ability to start, stop, and monitor activities.
        '''
        self._activity_lock = threading.Lock()

        # loads the activity implementations
        self._activity_loader = ActivityLoader()

        # dictionary mapping ActivityID to (activity_handle, activity_object)
        self._running_activities = {}

    def start_activity(self, activity_type, json_goal, desired_activity_id):
        # find the python class that implements this activity
        activity_class = self._activity_loader.get_activity_class(activity_type)
        activity_goal_class = self._activity_loader.get_activity_goal_class(activity_type)

        # convert the goal from JSON to a ROS message
        activity_goal_dict = json.loads(json_goal)
        activity_goal = message_conversion.populate_instance(activity_goal_dict, activity_goal_class)

        rospy.loginfo('Starting activity with class: %s', str(type(activity_class)))
        with self._activity_lock:
            # create a unique id for this activity
            activity_id = generate_unique_id(self._running_activities.keys(), desired_activity_id)

            # create an instance of this activity class
            activity_handle = ActivityHandle(activity_id)
            activity_object = activity_class(activity_handle, activity_goal)
            self._running_activities[activity_id] = (activity_handle, activity_object, activity_type)

            # start the activity in a separate thread
            activity_object.start()
        rospy.loginfo('Done starting activity')
            
        return activity_id

    def stop_activity(self, activity_id):
        rospy.loginfo('Stopping activity with id: %s', str(activity_id))
        with self._activity_lock:
            activity_handle, activity_object, activity_type = self._running_activities[activity_id]
        activity_handle.request_stop()

    def get_running_activities(self):
        self.update_running_activities()
        
        activity_infos = []
        with self._activity_lock:
            for activity_id in self._running_activities:
                activity_info = RunningActivityInfo()
                activity_handle, activity_object, activity_type = self._running_activities[activity_id]
                activity_info.activity_type = activity_type
                activity_info.activity_id = activity_id
                activity_infos.append(activity_info)
        return activity_infos

    def update_running_activities(self):
        '''
        Checks all activities that were running to see if they have finished.
        '''
        with self._activity_lock:
            for activity_id in self._running_activities.keys():
                activity_handle, activity_object, activity_type = self._running_activities[activity_id]
                
                if not activity_object.is_alive():
                    # the thread for this activity completed
                    del self._running_activities[activity_id] 

