import numpy as np
from tf import transformations

def pose_to_array(pose):
    '''
    Convert a pose message to a 4x4 numpy array.
    
    **Args:**
    
    **pose (geometry_msgs.msg.Pose):** Pose rospy message class.
    
    **Returns:**
    mat (numpy.array): 4x4 numpy array
    '''
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    pos = np.array([pose.position.x, pose.position.y, pose.position.z]).T
    mat = np.array(transformations.quaternion_matrix(quat))
    mat[0:3, 3] = pos
    return mat 

def array_to_pose(arr):
    '''
    Convert a homogeneous matrix to a Pose message, optionally premultiply by a transform.
    
    **Args:**
    
    **arr (numpy.ndarray):** 4x4 array (or matrix) representing a homogenous transform.
    
    *transform (numpy.ndarray):* Optional 4x4 array representing additional transform
    
    **Returns:**
    pose (geometry_msgs.msg.Pose): Pose message representing transform.
    '''
    pose = Pose()
    pose.position.x = mat[0,3]
    pose.position.y = mat[1,3]
    pose.position.z = mat[2,3]
    quat = transformations.quaternion_from_matrix(arr)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose 

def transform_to_array(transform):
    '''
    Convert a tf transform to a 4x4 scipy array.
    
    **Args:**
    
    **transform (geometry_msgs.msg.Transform):** Transform rospy message class.
    
    **Returns:**
    mat (numpy.array): 4x4 numpy array
    '''
    quat = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    pos = np.array([transform.translation.x, transform.translation.y, transform.translation.z]).T
    arr = np.array(transformations.quaternion_matrix(quat))
    arr[0:3, 3] = pos
    return arr
