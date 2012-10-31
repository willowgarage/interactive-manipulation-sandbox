#! /usr/bin/env python

import roslib
roslib.load_manifest('smach_executer')
import rospy
import actionlib
from executer_actions.msg import ExecuteAction

rospy.init_node('cancel_executer_goals')

client = actionlib.SimpleActionClient('/executer/execute', ExecuteAction)
client.wait_for_server()
client.cancel_all_goals()

rospy.sleep(1.0)

