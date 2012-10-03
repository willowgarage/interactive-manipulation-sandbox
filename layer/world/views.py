from django.http import HttpResponse
from django.shortcuts import *
import json
from layer.world.models import Place

# Create your views here.

def index(request):
    return render_to_response('index.html', {'robots':[
	{'name':'PRL', 'description':'PR-2 in the Green Room'},
	{'name':'TA101', 'description':'Production Texai'}]})

"""
JAC: unused

def image(request):
    place = Place.objects.all()[0]
    return render_to_response('image.html', locals())
"""
