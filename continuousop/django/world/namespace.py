import copy

from socketio.namespace import BaseNamespace
from sockets.health_monitor import HealthMonitorMixin


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

    # The following four events correspond to the NavigateToPose actions.

    def on_move_forward(self):
        action = copy.deepcopy(self.action)
        action['inputs']['x'] = 0.2
        print "COMMAND CAPTURED: move forward"  # DEBUG

    def on_move_back(self):
        action = copy.deepcopy(self.action)
        action['inputs']['x'] = -0.2
        print "COMMAND CAPTURED: move backward"  # DEBUG

    def on_turn_left(self):
        action = copy.deepcopy(self.action)
        action['inputs']['theta'] = 0.2
        print "COMMAND CAPTURED: turn left"  # DEBUG

    def on_turn_right(self):
        action = copy.deepcopy(self.action)
        action['inputs']['theta'] = -0.2
        print "COMMAND CAPTURED: turn right"  # DEBUG

    def on_stop_moving(self):
        print "COMMAND CAPTURED: stop moving"  # DEBUG
