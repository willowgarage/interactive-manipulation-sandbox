import threading
import numpy as np
import rospy

from pr2_msgs.msg import BatteryServer

class BatteryMonitor:
    def __init__(self):
        self._battery_topic = '/battery/server'
        self._battery_sub = rospy.Subscriber(self._battery_topic, BatteryServer, self.msg_callback)

        self._last_msg = None
        self._last_msg_lock = threading.Lock()

    def msg_callback(self, msg):
        with self._last_msg_lock:
            self._last_msg = msg

    def wait_for_first_update(self):
        while self._last_msg is None:
            rospy.logwarn('Waiting for first battery status message on %s' % self._battery_topic)
            rospy.sleep(0.01)

    def is_charging(self):
        self.wait_for_first_update()

        # the message's charging field is a uint16 with
        # no explanation of what it means; i'm assuming that
        # it will be nonzero if chargin and zero otherwise
        return bool(self._last_msg.charging)

    def time_left(self):
        self.wait_for_first_update()

        if self.is_charging():
            return np.inf

        return float(self._last_msg.timeLeft)

    
        
        
