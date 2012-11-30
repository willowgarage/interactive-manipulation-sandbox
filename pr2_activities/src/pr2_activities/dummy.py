import rospy
from activity_manager import ActivityBase

class Dummy(ActivityBase):
    def __init__(self, handle, goal):
        ActivityBase.__init__(self)
        self._handle = handle
        self._goal = goal

    def run(self):
        rospy.loginfo('Dummy activity: message: "%s"' % self._goal.message)
        for loop_i in range(10):
            rospy.loginfo('Dummy activity loop %d' % loop_i)
            rospy.sleep(0.5)
            if self._handle.get_stop_requested():
                rospy.loginfo('Dummy activity stop requested, stopping')
                break

    def stop(self):
        pass
