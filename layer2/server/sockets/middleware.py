# DISABLED: This middleware is currently unused
# It is left here as an example in case we want in the future to
# direct traffic to socket code before going through django's mechanism
from gevent import monkey
monkey.patch_all()

import os, copy

from socketio import socketio_manage
from sockets.namespace import ClientNamespace
from django.http import HttpResponseNotFound


# The namespaces served by the socket.io backend.
# Currently emtpy, due to the fact that gevent-socketio (as socket.io.js)
# implements heartbeats/reconnect (all that's needed for now).
namespaces = dict()

class GZipRequireJSHack(object):
    """
    This middleware is a disposable hack to allow serving pre-gzipped files
    to require.js, which won't accept using a different file extension other than .js
    through Django's StaticFilesHandler, which will only set the proper encoding headers
    to files which end with .gz
    We should remove this hack as soon as we move the static files outside of Django
    """
    def __init__(self, app):
        self.app = app

    def __call__(self, environ, start_response):
        saved_environ = copy.copy(environ)

        # First try to serve the original request
        result = self.app(environ, start_response)

        # If it fails and it is a .js file, try adding .gz and see what happens
        if isinstance(result, HttpResponseNotFound):
            path = saved_environ['PATH_INFO']
            if path.endswith(".js"):
                newpath = path.replace(".js",".js.gz")
                print "-------> Attempting to replace %s with gzipped equivalent %s" % (path, newpath)
                saved_environ['PATH_INFO'] = newpath
                result = self.app(saved_environ, start_response)

        return result

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
