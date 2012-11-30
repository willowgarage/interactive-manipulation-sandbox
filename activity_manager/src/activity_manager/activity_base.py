import threading

class ActivityBase(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        raise NotImplementedError("Activity's run routine is not defined")
