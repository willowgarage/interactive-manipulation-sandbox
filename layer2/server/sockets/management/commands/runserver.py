from gevent import monkey
monkey.patch_all()

from optparse import make_option
import re

from django.core.management.base import BaseCommand, CommandError
from django.core.wsgi import get_wsgi_application
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


naiveip_re = re.compile(r"""^(?:
(?P<addr>
(?P<ipv4>\d{1,3}(?:\.\d{1,3}){3}) |         # IPv4 address
(?P<fqdn>[a-zA-Z0-9-]+(?:\.[a-zA-Z0-9-]+)*) # FQDN
):)?(?P<port>\d+)$""", re.X)

DEFAULT_PORT = '8000'
DEFAULT_ADDR = '0.0.0.0'

class Command(BaseCommand):
    help = 'Server for Django which includes Socket.io support.'
    args = '[optional port number, or ipaddr:port]'

    def handle(self, addrport='', *args, **options):
        if args:
            raise CommandError('Usage is runserver %s' % self.args)
        if not addrport:
            self.host = DEFAULT_ADDR
            self.port = DEFAULT_PORT
        else:
            # The following code is a simplification of
            # django.core.management.commands.runserver.
            m = re.match(naiveip_re, addrport)
            if m is None:
                raise CommandError('"%s" is not a valid port number '
                                   'or address:port pair.' % addrport)
            self.host, _ipv4, _fqdn, self.port = m.groups()
            if not self.host:
                self.host = DEFAULT_ADDR
            if not self.port.isdigit():
                raise CommandError("%r is not a valid port number." % self.port)

        application = get_wsgi_application()

        # If on a development environment, serve static files a-la 'runserver'.
        if settings.DEBUG:
            # Add another middleware to the stack, to detour static file requests.
            from django.contrib.staticfiles.handlers import StaticFilesHandler
            application = StaticFilesHandler(application)

        # Add the SocketIO escape for requests.
        application = WithSocketIO(application)

        print
        print 'Listening on port %s:%s' % (self.host, self.port)
        print
        SocketIOServer((self.host, int(self.port)), application,

                       # Number of seconds between heartbeats from server to client.
                       heartbeat_interval=3,

                       # Number of seconds to wait for a heartbeat. If this
                       # timeout is not met, the connection is considered lost.
                       heartbeat_timeout=10,

                       resource=WithSocketIO.resource,
                       policy_server=False).serve_forever()
