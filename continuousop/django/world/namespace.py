from socketio.namespace import BaseNamespace
from sockets.health_monitor import HealthMonitorMixin
from robot import RobotProxy

import logging
logger = logging.getLogger('robot')


class Namespace(BaseNamespace, HealthMonitorMixin):

    # The namespace to expose to socket.io client code.
    namespace = '/robot'

    def initialize(self):
        self.robot = RobotProxy()

    def recv_disconnect(self):
        if self.robot.moving:
            self.robot.stop_moving()

    def robot_stopped_moving(self):
        """Inform the client the robot has stoped moving."""
        self.emit('robot stopped')

    # The following five events correspond to the NavigateToPose actions.

    def on_move_forward(self):
        logger.debug('NAMESPACE: moving the robot forward.')
        self.robot.move('move forward')

    def on_move_back(self):
        logger.debug('NAMESPACE: moving the robot backwards.')
        self.robot.move('move back')

    def on_turn_left(self):
        logger.debug('NAMESPACE: turning the robot left.')
        self.robot.move('turn left')

    def on_turn_right(self):
        logger.debug('NAMESPACE: turning the robot right.')
        self.robot.move('turn right')

    def on_stop_moving(self):
        logger.debug('NAMESPACE: stopping the robot.')
        self.robot.stop_moving(self.robot_stopped_moving)
