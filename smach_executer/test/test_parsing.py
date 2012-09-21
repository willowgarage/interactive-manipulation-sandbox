PACKAGE_NAME = 'smach_executer'
import roslib; roslib.load_manifest(PACKAGE_NAME)
import os.path
import json

from smach_executer import parser
from smach_executer.action import Action
from smach_executer.actions.dummy_action import DummyAction

# find directories we need
package_root_dir = roslib.packages.get_pkg_dir(PACKAGE_NAME)
json_dir = os.path.join(package_root_dir, 'test/json')

def test_parse_empty_dict():
    ''' Expect a ValueError when parsing an empty dicitionary '''
    try:
        a = parser.create_state_machine_from_action_dict({}, {})
    except ValueError:
        return

def test_dummy_action():
    ''' Parse a simple action from JSON '''
    json_filename = os.path.join(json_dir, 'execute_dummy_action.js')
    request_str = open(json_filename).read()
    request_dict = json.loads(request_str)
    sm = parser.create_state_machine_from_action_dict(
        request_dict['action'], {'Dummy':Dummy})
    sm.execute()

