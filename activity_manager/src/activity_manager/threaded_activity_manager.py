import rospy
import threading
from activity_manager.activity_handle import ActivityHandle


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

        # dictionary mapping ActivityID to (activity_handle, activity_object)
        self._running_activities = {}

    def start_activity(self, activity_class, goal, desired_activity_id):
        rospy.loginfo('Starting activity with class: %s', str(type(activity_class)))
        with self._activity_lock:
            # make sure to use a unique id
            activity_id = generate_unique_id(self._running_activities.keys(), desired_activity_id)

            # create an instance of this activity class
            activity_handle = ActivityHandle(activity_id)
            activity_object = activity_class(activity_handle, goal)
            self._running_activities[activity_id] = (activity_handle, activity_object)

            # start the activity in a separate thread
            activity_object.start()
        rospy.loginfo('Done starting activity')
            
        return activity_id

    def stop_activity(self, activity_id):
        rospy.loginfo('Stopping activity with id: %s', str(activity_id))
        with self._activity_lock:
            activity_handle, activity_object = self._running_activities[activity_id]
        activity_handle.request_stop()
