from socketio.namespace import BaseNamespace
import redis
import json

r = redis.Redis('localhost')

class ClientNamespace(BaseNamespace):

    '''Called by the client to indicate the user has navigated to another
       page or context within the client application'''
    def on_context_new(self, new_context):
        # In order to implement this, we use a key store with:
        # - HASH context: set of users in this context. Each user entry contains: userid and names
        # Information to the client will go by registering to updates in the 'context' key
        self.log("New context: %s" % new_context)

        # TODO: I don't know how to store an instance variable that is kept in the context
        # of this namespace instance for this particular connection so that I can get ahold
        # of it from a different socket session (so as to emit updates to others, for example)
        # so I ended up using redis for this mapping, which I think is overkill
        # TODO: Investigate further the relationship between session.session_key and sessid
        old_context = r.get('sessid:%s' % self.socket.sessid)
        r.set('sessid:%s' % self.socket.sessid, new_context)

        # Delete and notify old context if there is one
        if old_context:
            r.hdel('context:%s' % old_context, self.socket.sessid)
            self.update_contexts( old_context)

        # Update and notify new value
        r.hset('context:%s' % new_context, self.socket.sessid, json.dumps(make_user(self.request.user)))
        self.update_contexts( new_context)

    # Remove from the current context on disconnect (otherwise phantom users keep piling up)
    def recv_disconnect( self):
        self.log("disconnect")
        old_context = r.get('sessid:%s' % self.socket.sessid)
        if old_context is not None:
            r.hdel('context:%s' % old_context, self.socket.sessid)
            self.update_contexts( old_context)
            r.delete('sessid:%s' % self.socket.sessid)

    def log( self, message):
        print "(socket %s) %s" % (self.socket.sessid, message)

    '''Send a message to all the connected clients which are in the context being updated'''
    def update_contexts( self, context):
        # all_clients keeps a hash with all the users connected to this context
        all_clients = r.hgetall('context:%s' % context)
        # Deserialize (I don't know why the redis API doesn't do this automatically)
        for k, v in all_clients.items():
            self.log("deserializing: %s" % v)
            all_clients[k] = json.loads(v)

        # Go through all connected sockets and send updates to clients connected to this context
        for sessid, socket in self.socket.server.sockets.iteritems():
            sessid_context = r.get('sessid:%s' % sessid)

            # If this session is in the updated context, send the update to this user
            if sessid_context == context:
                # Craft the update data for each user:
                # 1- Make a copy of the has of all connected users
                sessid_others = all_clients.copy()

                # 2- Remove this particular user from the update (it wants to know which _other_ users are connected)
                try:
                    del sessid_others[sessid]
                except KeyError:
                    print "Logic error. Attempting to send update to sessid = [%s] supposedly in context = [%s] but didn't find key for that sessid in key store hash with that context key" % ( sessid, context)

                # 3- Send them in the form of a list
                list_others = sessid_others.values()

                # Send the update
                self.log("emitting context_others event with %s" % list_others)
                socket["/client"].emit('context_others', list_others)
        
def make_user( user):
    if user.username == '':
        return {
            'username': 'AnonymousUser',
            'first_name': '',
            'last_name': 'Anonymous'
        }
    else:
        return {
            'username': user.username,
            'first_name': user.first_name,
            'last_name': user.last_name
        }

