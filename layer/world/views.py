from django.http import HttpResponse
from django.shortcuts import *
import json

# Create your views here.

def index(request):
    return render_to_response('index.html', {'robots':[
	{'name':'PRL', 'description':'PR-2 in the Green Room'},
	{'name':'TA101', 'description':'Production Texai'}]})

