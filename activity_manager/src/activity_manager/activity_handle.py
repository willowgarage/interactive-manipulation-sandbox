import threading

class ActivityHandle:
    def __init__(self, activity_id):
        self._lock = threading.Lock()
        self._stop_requested = False
        self._activity_id = activity_id

    def get_stop_requested(self):
        with self._lock:
            return self._stop_requested

    def get_activity_id(self):
        return self._activity_id

    def request_stop(self):
        with self._lock:
            self._stop_requested = True
