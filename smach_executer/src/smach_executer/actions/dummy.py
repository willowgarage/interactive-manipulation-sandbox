import rospy
import smach

SUCCEEDED = 'succeeded'
FAILED = 'failed'

class Dummy(smach.State):
    def __init__(self, outcomes=[SUCCEEDED, FAILED] ):
        smach.State.__init__(self, outcomes=outcomes)

    def execute(self, userdata):
        rospy.loginfo('Hi, I\'m the dummy action!')
        return SUCCEEDED
        
