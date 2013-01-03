'''
This django view is only used to link request to gevent-socketio namespaces
'''
from socketio import socketio_manage
from namespace import ClientNamespace
from sockets.latency_monitor import LatencyMonitor

def socketio(request):
        socketio_manage(request.environ,{
                '/client': ClientNamespace,
                '/monitor': LatencyMonitor,
        }, request)
