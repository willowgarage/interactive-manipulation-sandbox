from gevent import monkey
monkey.patch_all()

from optparse import make_option

from django.core.management.base import BaseCommand
from django.conf import settings

from socketio import socketio_manage
from socketio.server import SocketIOServer


# The namespaces served by the socket.io backend.
# Currently emtpy, due to the fact that gevent-socketio (as socket.io.js)
# implements heartbeats/reconnect (all that's needed for now).
namespaces = dict()


class WithSocketIO(object):
    """WSGI middleware to provide differential service to socket requests.

    Requests comming to "socket.io" resource are served to the socketio_manage
    WSGI application, all others to the Django application.
    """

    resource = 'socket.io'

    # Note about the resource name:
    #
    # Changing the resource name involves changes in three points.
    #
    # 1) This middleware check, by way of path.startswith or similar.
    #
    # 2) The SocketIOServer instance must be passed the 'namespace' keyword.
    #
    # 3) The client should call io.connect(url, {resource: "the-new-name"})
    #
    # The concept of "namespace" is unique to gevent-socketio, sometimes
    # confused to that of "resource" (which is shared with socket.io.js), and
    # is the result of a rather silly name problem in the python library. For
    # details about the resource-namespace dichotomy (and other, interesting,
    # notes), see https://github.com/abourget/gevent-socketio/issues/39.

    def __init__(self, app):
        self.app = app

    def __call__(self, environ, start_response):
        path = environ['PATH_INFO'].strip('/')

        if path.startswith(WithSocketIO.resource):
            socketio_manage(environ, namespaces)
        else:
            return self.app(environ, start_response)


###############################################################################


class Command(BaseCommand):
    help = 'Server for Django which includes Socket.io support.'

    option_list = BaseCommand.option_list + (
        make_option(
            '--port',
            action='store',
            dest='port',
            default=8000,
            type='int',
            help='Port used for incomings requests -- default to 8000'),
        make_option(
            '--host',
            action='store',
            dest='host',
            default='0.0.0.0',
            help='Host -- defaults to 0.0.0.0'),)

    def handle(self, **options):
        import os

        # TODO: is this necessary? If so, can the settings path, or at least
        # theproject name, be obtained from python somehow?  It is very ugly to
        # see this hardcoded here.
        PROJECT_SETTINGS = "server.settings"
        os.environ.setdefault("DJANGO_SETTINGS_MODULE", PROJECT_SETTINGS)

        from django.core.wsgi import get_wsgi_application
        application = get_wsgi_application()

        print
        print 'Listening on port %s:%s' % (options['host'], options['port'])
        print
        SocketIOServer((options['host'], options['port']),
                       WithSocketIO(application),

                       # Number of seconds between heartbeats from server to client.
                       heartbeat_interval=3,

                       # Number of seconds to wait for a heartbeat. If this
                       # timeout is not met, the connection is considered lost.
                       heartbeat_timeout=10,

                       resource=WithSocketIO.resource,
                       policy_server=False).serve_forever()
