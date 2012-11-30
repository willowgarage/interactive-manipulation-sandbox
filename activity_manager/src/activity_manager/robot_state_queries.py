import copy
import tf2

from activity_manager.joint_state_listener import JointStateListener

initialized = False
transform_listener = None
joint_state_listener = None

def initialize():
    '''
    Sets up objects that are used by the query functions.

    This MUST be called before evaluating queries.
    '''
    global transform_listener, joint_state_listener, initialized
    transform_listener = tf2.TransformListener()
    joint_state_listener = JointStateListener()
    initialized = True

def pose_to_frame(pose, current_header, desired_header):
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
    except (tf2.LookupException, tf2.ConnectivityException):
        continue



def pose_within_bbox(frame_name, pose):
    '''
    Args:
        frame_name (str): TF frame to do check in.
        pose (geometry_msgs.msg.Pose): pose to check
