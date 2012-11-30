'''
Action for segmenting and recognizing objects
'''
import rospy
import actionlib
import threading
import scipy
from smach import State

from pr2_interactive_object_detection.msg import UserCommandAction, UserCommandGoal
from object_manipulation_msgs.msg import GraspableObjectList
from actionlib_msgs.msg import GoalStatus
from object_manipulator.convert_functions import *

class SegmentAndRecognize(State):
    """
    Segments and recognize objects (well-separated, on a table)
    Assumes you're already looking at the objects

    * 'succeeded' Finished without error.
    * 'failed' Something went wrong.
    * 'preempted' Somebody preempted.
    """

    def __init__(self, input_keys=[]):
        action_uri = 'object_detection_user_command'
        self.detection_client = actionlib.SimpleActionClient(action_uri, UserCommandAction)
        rospy.loginfo("waiting for %s"%action_uri)
        self.detection_client.wait_for_server()
        rospy.loginfo("%s found"%action_uri)

        #state flag and lock for changing flag
        self.state = None
        self.lock = threading.Lock()

        #normal init for smach States
        State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'], input_keys = input_keys, output_keys = ['outputs'])

        self.segmentation_timeout = rospy.get_param('segmentation_timeout', 10.)
        self.recognition_timeout = rospy.get_param('recognition_timeout', 10.)

        #listener for the results
        rospy.Subscriber("interactive_object_recognition_result", GraspableObjectList, self.process_results)
     
        #output dictionary to return
        self.outputs = None

        rospy.loginfo("done SegmentAndRecognize init")

                     
    def send_goal_and_wait(self, goal, timeout):
        self.detection_client.send_goal(goal)
        start_time = rospy.get_rostime()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            if now - start_time > rospy.Duration(timeout):
                rospy.loginfo("segment and recognize timed out!")
                self.detection_client.cancel_goal()
                return 'failed'
            if self.preempt_requested():
                rospy.loginfo("segment and recognize goal preempted!")
                self.detection_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            state = self.detection_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                return 'succeeded'
            elif state not in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                return 'failed'
            r.sleep()

  
    def execute(self, userdata):

        #first call segmentation
        goal = UserCommandGoal()
        goal.request = goal.SEGMENT
        goal.interactive = False
        rospy.loginfo("Sending segmentation result and waiting for result")
        with self.lock: 
            self.state = "segmenting"
        result = self.send_goal_and_wait(goal, self.segmentation_timeout)
        if result != 'succeeded':
            return result

        #now call recognition
        rospy.loginfo("segmentation done, recognizing")
        goal.request = goal.RECOGNIZE
        with self.lock:
            self.state = "recognizing"
        result = self.send_goal_and_wait(goal, self.recognition_timeout)
        if result != 'succeeded':
            with self.lock:
                self.state = "done"

        #check if we're done processing the results
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested():
                rospy.loginfo("segment and recognize goal preempted after recognition!")
                self.service_preempt()
                userdata.outputs = None
                return 'preempted'
            with self.lock:
                if self.state == "error":
                    userdata.outputs = None
                    return 'failed'
                if self.state == "done":
                    userdata.outputs = self.outputs
                    return 'succeeded'
        

    #find the corners of the object bounding box in normalized image coords
    def find_normalized_bounding_box(self, x_vals, y_vals, z_vals, camera_info):
        
        #bounding box corners in camera frame coords (Euclidean)
        z_center = (min(z_vals) + max(z_vals))/2.

        #pinhole model camera params
        fx = camera_info.P[0*4+0]
        fy = camera_info.P[1*4+1]
        cx = camera_info.P[0*4+2]
        cy = camera_info.P[1*4+2]

        #convert bounding box corners to normalized image coords
        u_min = cx + min(x_vals) / z_center * fx
        u_max = cx + max(x_vals) / z_center * fx
        v_min = cy + min(y_vals) / z_center * fy
        v_max = cy + max(y_vals) / z_center * fy
        normalized_box = {'xmin': u_min / camera_info.width,
                          'ymin': v_min / camera_info.height,
                          'xmax': u_max / camera_info.width,
                          'ymax': v_max / camera_info.height}
        return normalized_box


    #find the bounding boxes in the camera frame 
    def process_results(self, msg):

        #ignore segmentation results (waiting for recognition results)
        with self.lock:
            if self.state != "recognizing":
                return

        #find the object's bounding box in the camera image
        object_dict = {}
        object_ind = 0
        for (graspable_object, mesh) in zip(msg.graspable_objects, msg.meshes):
            x_values = []
            y_values = []
            z_values = []

            #transform from object frame to camera
            ref_to_camera_mat = pose_to_mat(msg.reference_to_camera)

            #recognized object mesh, transform vertices into camera frame
            if len(mesh.vertices) > 0:
                for vertex in mesh.vertices:
                    camera_frame_point = ref_to_camera_mat * scipy.matrix(get_xyz(vertex) + [1]).T
                    x_values.append(camera_frame_point[0,0])
                    y_values.append(camera_frame_point[1,0])                
                    z_values.append(camera_frame_point[2,0])

            #segmented object point cloud, transform points into camera frame
            elif len(graspable_object.cluster.points) > 0:
                for point in graspable_object.cluster.points:
                    camera_frame_point = ref_to_camera_mat * scipy.matrix(get_xyz(point) + [1]).T
                    x_values.append(camera_frame_point[0,0])
                    y_values.append(camera_frame_point[1,0])
                    z_values.append(camera_frame_point[2,0])

            #empty object?
            else:
                self.state = "error"
                return

            #get the bounding box in normalized coordinates
            bounding_box = self.find_normalized_bounding_box(x_values, y_values, z_values, msg.camera_info)

            #return bounding box for object number object_ind
            object_dict[object_ind] = bounding_box
            object_ind += 1

        self.outputs = object_dict

        with self.lock:
            self.state = "done"

            


