from django.conf.urls.defaults import patterns, include, url
from django.contrib.auth.models import AnonymousUser
from djangorestframework.resources import ModelResource
from djangorestframework.views import *
from djangorestframework.mixins import ListModelMixin
from layer.restmodels import ReadOnlyModelView
from models import Place, Robot

class PlaceModel(ModelResource):
    model = Place
    exclude = ()    # This was necessary to override the exclusion of the 'id' field
class RobotModel(ModelResource):
    model = Robot
    exclude = ()    # This was necessary to override the exclusion of the 'id' field

# This view filters the robots that are returned to the user
# If the user is logged in, then all robots are returned
# Otherwise, only Robots which are marked with property "Public" = True
class RobotsView(View, ListModelMixin):
    def get(self, request, *args, **kwArgs):
        if type(request.user) == AnonymousUser:
            kwArgs['public'] = True
        return super(RobotsView, self).get(request, *args, **kwArgs)

urlpatterns = patterns('',
    url(r'^places$', ListModelView.as_view(resource=PlaceModel)),
    url(r'^places/(?P<pk>[^/]+)/$', ReadOnlyModelView.as_view(resource=PlaceModel)),
    url(r'^places/admin$', ListOrCreateModelView.as_view(resource=PlaceModel)),
    url(r'^places/admin/(?P<pk>[^/]+)/$', InstanceModelView.as_view(resource=PlaceModel)),

    url(r'^robots$', RobotsView.as_view(resource=RobotModel)),
    url(r'^robots/(?P<pk>[^/]+)/$', ReadOnlyModelView.as_view(resource=RobotModel)),
    url(r'^robots/admin$', ListOrCreateModelView.as_view(resource=RobotModel)),
    url(r'^robots/admin/(?P<pk>[^/]+)/$', InstanceModelView.as_view(resource=RobotModel)),
)
