import copy, re
import numpy as np
import json
import rospy
import tf

from geometry_msgs.msg import Pose, Transform, Vector3, Quaternion, Point
from std_msgs.msg import Header

# used to convert between json strings and ROS messages
from rosbridge_library.internal import message_conversion

from activity_manager.joint_state_listener import JointStateListener
from activity_manager.battery_monitor import BatteryMonitor
from activity_manager.query_parser import QueryFunctionCall, parse_query
import activity_manager.conversions as conv

simple_types = [str, int, float, bool]

def to_json(msg):
    if msg.__class__ in simple_types:
        return json.dumps(msg)
    print msg
    return json.dumps(message_conversion.extract_values(msg))

def from_json(json_str, message_class):
    json_val = json.loads(json_str)
    if message_class in simple_types:
        return json_val
    return message_conversion.populate_instance(json_val, message_class)

class QueryFunction:
    def __init__(self, func, arg_types):
        self.func = func
        self.arg_types = arg_types

class QueryEngine:
    def __init__(self):
        self._transform_listener = tf.TransformListener()
        self._joint_state_listener = JointStateListener()
        self._battery_monitor = BatteryMonitor()
        self._query_functions = {
            'get_transform' : QueryFunction(self.get_transform, [str, str]),
            'transform_point_to_frame' : QueryFunction(self.transform_point_to_frame, [Point, str, str]),
            'point_within_bbox' : QueryFunction(self.point_within_bbox, [Point, Point, Vector3]),
            'is_charging' : QueryFunction(self.is_charging, []),
            'and' : QueryFunction(self.logical_and, [bool, bool]),
            'or' : QueryFunction(self.logical_or, [bool, bool])
            }
        
    def get_query_functions(self):
        return self._query_functions

    def evaluate_query(self, query, args):
        rospy.loginfo('Query: "%s"' % query)
        # parse the query
        parsed_query = parse_query(query)
        rospy.loginfo('Parsed query: "%s"' % parsed_query)

        # create a map of argument names -> values
        arg_dict = {}
        for arg in args:
            arg_dict[arg.name] = arg.value

        # evaluate the query
        res = self.evaluate_expression(parsed_query, arg_dict)

        # convert the result of the query into JSON
        return to_json(res)

    def evaluate_expression(self, expression, arg_dict):
        if isinstance(expression, QueryFunctionCall):
            return self.evaluate_function(expression, arg_dict)
        else:
            raise ValueError('Unknown expression type')

    def evaluate_function(self, func_call, arg_dict):
        func_name = func_call.name        
        rospy.loginfo('Evaluating function %s' % func_name)

        if func_name not in self._query_functions:
            raise ValueError('Unknown query function %s' % func_name)

        func = self._query_functions[func_name]

        # assuming for now that arg is a variable, not the result of another expression
        args = []
        for arg_i, arg in enumerate(func_call.args):
            if isinstance(arg, QueryFunctionCall):
                # this argument is another function, recurse and evaluate it
                arg_val = self.evaluate_function(arg, arg_dict)
                args.append(arg_val)
            elif isinstance(arg, str):
                # this argument is a variable (as json). convert it to a ROS msg type
                try:
                    arg_json_str = arg_dict[arg]
                except KeyError:
                    raise ValueError('No value specified for query arg %s' % arg)
                
                rospy.loginfo('   arg %s: %s' % (arg, arg_json_str))

                # lookup the class for this argument, using the QueryFunction declaration
                arg_class = func.arg_types[arg_i]                                                 

                # convert the value for this argument from JSON to a ROS message
                args.append(from_json(arg_json_str, arg_class()))
            else:
                raise ValueError('Improper argument type for function')

        # evaluate the function
        return func.func(*args)

    def get_transform(self, current_frame, desired_frame):
        try:
            (trans, rot) = self._transform_listener.lookupTransform(
                current_frame, desired_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            raise
        return Transform(Vector3(*trans), Quaternion(*rot))

    def transform_point_to_frame(self, point, current_frame, desired_frame):
        point_arr_h = np.array([point.x, point.y, point.z, 1.0])
        transform_arr = conv.transform_to_array(self.get_transform(current_frame, desired_frame))
        new_point_arr_h = np.dot(transform_arr, point_arr_h)
        new_point_arr = new_point_arr_h[:3] / new_point_arr_h[3]
        return Point(*new_point_arr)

    def is_charging(self):
        return self._battery_monitor.is_charging()

    def logical_and(self, arg1, arg2):
        return arg1 and arg2

    def logical_or(self, arg1, arg2):
        return arg1 or arg2

    def point_within_bbox(self, point, bbox_origin, bbox_size):
        '''
        Args:
            point - geometry_msgs.msg.Point
            bbox_origin - geometry_msgs.msg.Point
            bbox_size - geometry_msgs.msg.Vector3

        Returns:
            (bool) - whether point is within bounding box.
        '''
        return ((point.x >= bbox_origin.x) and
                (point.y >= bbox_origin.y) and
                (point.z >= bbox_origin.z) and
                (point.x <= (bbox_origin.x + bbox_size.x)) and
                (point.y <= (bbox_origin.y + bbox_size.y)) and
                (point.z <= (bbox_origin.z + bbox_size.z)))
                  
