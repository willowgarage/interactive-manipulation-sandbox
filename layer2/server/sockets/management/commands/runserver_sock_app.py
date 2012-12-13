from gevent import monkey
monkey.patch_all()

from optparse import make_option

from django.core.management.base import BaseCommand
from django.conf import settings

from socketio import socketio_manage
from socketio.server import SocketIOServer


class WithSocketIO(object):

    def __init__(self, app):
        self.app = app

    def __call__(self, environ, start_response):
        path = environ['PATH_INFO'].strip('/')

        if path.startswith('socket.io'):
            socketio_manage(environ)
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

        # TODO: obtain the project name from python; avoid hardcoding it here.
        os.environ.setdefault("DJANGO_SETTINGS_MODULE", "server.settings")

        from django.core.wsgi import get_wsgi_application
        application = get_wsgi_application()

        print
        print 'Listening on port %s:%s' % (options['host'], options['port'])
        print
        SocketIOServer((options['host'], options['port']),
                       WithSocketIO(application),

                       heartbeat_interval=3,
                       heartbeat_timeout=10,

                       resource="socket.io",
                       policy_server=False).serve_forever()
