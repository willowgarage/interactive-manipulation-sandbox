from django.conf.urls.defaults import patterns, include, url
from djangorestframework.resources import ModelResource
from djangorestframework.views import *
from layer.restmodels import ReadOnlyModelView
from models import Place, Robot

class PlaceModel(ModelResource):
    model = Place
    exclude = ()    # This was necessary to override the exclusion of the 'id' field
class RobotModel(ModelResource):
    model = Robot
    exclude = ()    # This was necessary to override the exclusion of the 'id' field

urlpatterns = patterns('',
    url(r'^places$', ListModelView.as_view(resource=PlaceModel)),
    url(r'^places/(?P<pk>[^/]+)/$', ReadOnlyModelView.as_view(resource=PlaceModel)),
    url(r'^places/admin$', ListOrCreateModelView.as_view(resource=PlaceModel)),
    url(r'^places/admin/(?P<pk>[^/]+)/$', InstanceModelView.as_view(resource=PlaceModel)),

    url(r'^robots$', ListModelView.as_view(resource=RobotModel)),
    url(r'^robots/(?P<pk>[^/]+)/$', ReadOnlyModelView.as_view(resource=RobotModel)),
    url(r'^robots/admin$', ListOrCreateModelView.as_view(resource=RobotModel)),
    url(r'^robots/admin/(?P<pk>[^/]+)/$', InstanceModelView.as_view(resource=RobotModel)),
)
