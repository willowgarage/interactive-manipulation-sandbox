import rospy
from smach_ros import SimpleActionState
from pr2_plugs_msgs.msg import RechargeGoal, RechargeCommand, RechargeAction

class Unplug(SimpleActionState):
    """
    Unplug from the wall.

    Preconditions: 
        Robot is plugged in, with its right gripper holding the plug.
        This is the state which the PlugIn action leaves the robot in.

    Outcomes:
    * 'succeeded':  The robot is unplugged, and the plug is on its magnetic holder.
    * 'failed': Unknown state.
    """
    def __init__(self, input_keys=[]):
        SimpleActionState.__init__(self, 'recharge', RechargeAction, goal_cb=self.goal_cb, input_keys=input_keys)

    def goal_cb(self, userdata, goal):
        goal = RechargeGoal()
        cmd = RechargeCommand()
        cmd.plug_id = 'local'
        cmd.command = RechargeCommand.UNPLUG
        goal.command = cmd
        return goal

