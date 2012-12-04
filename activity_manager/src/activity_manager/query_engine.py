import copy, re
import json
import rospy
import tf

from geometry_msgs.msg import Pose, Transform, Vector3, Quaternion, Point
from std_msgs.msg import Header

# used to convert between json strings and ROS messages
from rosbridge_library.internal import message_conversion

from activity_manager.joint_state_listener import JointStateListener
from activity_manager.query_parser import QueryFunctionCall, parse_query

def to_json(msg):
    if msg.__class__ in [str, int, float]:
        return json.dumps(msg)
    return json.dumps(message_conversion.extract_values(msg))

def from_json(json_str, message_class):
    json_val = json.loads(json_str)
    if message_class in [str, int, float]:
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
        self._query_functions = {
            'pose_in_frame' : QueryFunction(self.pose_in_frame, [Pose, Header, Header]),
            'get_transform' : QueryFunction(self.get_transform, [str, str])
            }

    def evaluate_query(self, query, args):
        rospy.loginfo('Query: "%s"' % query)
        # parse the query
        parsed_query = parse_query(query)
        rospy.loginfo('Parsed query: "%s"' % parsed_query)

        # create a map of argument names -> values
        arg_dict = {}
        for arg in args:
            arg_dict[arg.name] = arg.value
        print arg_dict

        # evaluate the query
        res = self.evaluate_expression(parsed_query, arg_dict)

        # convert the result of the query into JSON
        return to_json(res)

    def evaluate_expression(self, expression, arg_dict):
        if isinstance(expression, QueryFunctionCall):
            rospy.loginfo('Evaluating function call')
            return self.evaluate_function(expression, arg_dict)
        else:
            raise ValueError('Unknown expression type')

    def evaluate_function(self, func_call, arg_dict):
        func_name = func_call.name
        if func_name not in self._query_functions:
            raise ValueError('Unknown query function %s' % func_name)

        func = self._query_functions[func_name]

        # assuming for now that arg is a variable, not the result of another expression
        args = []
        for arg_i, query_variable in enumerate(func_call.args):
            # convert the argument value from JSON to a ROS message
            arg_name = query_variable.name
            arg_class = func.arg_types[arg_i]
            
            print '::', arg_name, arg_dict
            try:
                arg_json_str = arg_dict[arg_name]
            except KeyError:
                raise ValueError('No value specified for query arg %s' % arg_name)

            # convert the value for this argument from JSON to a ROS message
            arg_msg = from_json(arg_json_str, arg_class)
            args.append(arg_msg)

        # evaluate the function
        return func.func(*args)

    def get_transform(self, current_frame, desired_frame):
        try:
            (trans, rot) = self._transform_listener.lookupTransform(
                current_frame, desired_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            raise
        return Transform(Vector3(*trans), Quaternion(*rot))

    def pose_in_frame(self, pose, current_header, desired_header):
        '''
        Args:
            pose (geometry_msgs.msg.Pose) - Pose to transform.
            current_header (std_msgs.msg.Header) - Current header of pose.
            desired_header (std_msgs.msg.Header) - Desired header for pose.
            
        Returns:
            Pose in the frame/stamp of desired_header
        '''
        if current_header == desired_header:
            return pose
    
        try:
            (trans, rot) = transform_listener.lookupTransform(
                current_header.frame_id, current_header.stamp, desired_header.frame_id, desired_header.stamp)
        except (tf.LookupException, tf.ConnectivityException):
            raise

    def pose_within_bbox(self, frame_name, pose):
        '''
        Args:
            frame_name (str): TF frame to do check in.
            pose (geometry_msgs.msg.Pose): pose to check
        '''
