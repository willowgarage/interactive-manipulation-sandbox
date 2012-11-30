import rospy
import tf2

from sensor_msgs.msg import JointState

class TransformMonitor:
    def __init__(self):
        self._listener = tf2.TransformListener()

    def get_transform(self, from_frame, to_frame):
        try:
            (trans,rot) = self._listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf2.LookupException, tf2.ConnectivityException):
            continue
