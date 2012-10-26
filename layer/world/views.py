from django.http import HttpResponse
from django.shortcuts import *
from django.contrib.auth.decorators import login_required
from layer.world.models import *
import json

# Create your views here.

def index(request):
    return render_to_response('index.html', {'user': request.user})

def context(request):
    # Get or create Client object associated with this session
    print "Session Key = %s" % request.session.session_key
    try: 
        client = Client.objects.get(session_key=request.session.session_key)
        print "got existing"
    except Client.DoesNotExist:
        client = Client(session_key=request.session.session_key)
        print "created new"

    # Associate the current logged-in user to the session
    client.username = request.user.username

    # TODO: Associate current user context with client object

    client.save()

    # TODO: Search and return all other users currently in this context
    # TODO: Clean-up and remove stale client objects

    response_obj = {
        'username': client.username,
        'other_users': []
    }
    
    return HttpResponse(json.dumps(response_obj))
