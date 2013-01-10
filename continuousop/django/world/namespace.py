import copy
import logging

from socketio.namespace import BaseNamespace
from sockets.health_monitor import HealthMonitorMixin


logger = logging.getLogger('robot')


class Robot:
    """
    """
    def __init__(self):
        self.moving = False

    def move_as(self, action):
        """Sends command to the robot to move using "action"."""
        self.moving = True
        logger.debug("moving.... %s" % action)

    def stop_moving(self):
        """Sends command to robot to stop moving."""
        logger.debug("...stopping the robot...")
        self.stopped_moving()

    def stopped_moving(self):
        """Sends notice to the client that the robot is done."""
        self.moving = False
        logger.debug("...done moving.")


class Namespace(BaseNamespace, HealthMonitorMixin):

    # The namespace to expose to socket.io client code.
    namespace = '/robot'

    # Template action data for a NavigateToPose action.
    action = {
        'name': 'NavigateToPose',
        'inputs': {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'collision_aware': False,
            'frame_id': '/base_footprint',
        },
    }

    def initialize(self):
        self.robot = Robot()

    def recv_disconnect(self):
        if self.robot.moving:
            self.robot.stop_moving()

    # The following five events correspond to the NavigateToPose actions.

    def on_move_forward(self):
        action = copy.deepcopy(self.action)
        action['inputs']['x'] = 0.2
        self.robot.move_as(action)

    def on_move_back(self):
        action = copy.deepcopy(self.action)
        action['inputs']['x'] = -0.2
        self.robot.move_as(action)

    def on_turn_left(self):
        action = copy.deepcopy(self.action)
        action['inputs']['theta'] = 0.2
        self.robot.move_as(action)

    def on_turn_right(self):
        action = copy.deepcopy(self.action)
        action['inputs']['theta'] = -0.2
        self.robot.move_as(action)

    def on_stop_moving(self):
        self.robot.stop_moving()
