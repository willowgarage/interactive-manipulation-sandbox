from django.http import HttpResponse
from django.shortcuts import *
from django.contrib.auth.decorators import login_required

# Create your views here.

def index(request):
    return render_to_response('index.html', {'user': request.user})
