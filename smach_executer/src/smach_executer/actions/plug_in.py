import rospy
from smach_ros import SimpleActionState
from pr2_plugs_msgs.msg import RechargeGoal, RechargeCommand, RechargeAction

class PlugIn(SimpleActionState):
    """
    Plug into the wall automatically.

    Preconditions: Robot is a foot or two from the wall, and parallel with 
      its right side facing the wall. The PR2 must be using the special plug
      which has a fiducial on it, and this plug must be magnetically attached
      to the square gray patch on the top of the robot's base.

    Outcomes:
    * 'succeeded':  The robot is plugged in.
    * 'failed': Robot not necesarily plugged in.
    """
    def __init__(self, input_keys=[]):
        SimpleActionState.__init__(self, 'recharge', RechargeAction, goal_cb=self.goal_cb, input_keys=input_keys)

    def goal_cb(self, userdata, goal):
        goal = RechargeGoal()
        cmd = RechargeCommand()
        cmd.plug_id = 'local'
        cmd.command = RechargeCommand.PLUG_IN
        goal.command = cmd
        return goal
