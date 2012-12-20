from django.conf.urls.defaults import patterns, include, url
from django.contrib.auth.models import AnonymousUser
from rest_framework import generics, serializers
from models import Place, Robot, Camera

# This view filters the robots that are returned to the user
# If the user is logged in, then all robots are returned
# Otherwise, only Robots which are marked with property "Public" = True
class PrivateView(generics.ListAPIView):
    def get(self, request, *args, **kwArgs):
        if request.user.username == '':
            self.queryset = Robot.objects.filter(public=True)
        return super(PrivateView, self).get(request, *args, **kwArgs)

class CameraSerializer(serializers.ModelSerializer):
    class Meta:
        model = Camera
        exclude = ('id',)

class RobotSerializer(serializers.ModelSerializer):
    cameras = CameraSerializer(source='cameras')
    class Meta:
        model = Robot

urlpatterns = patterns('',
    url(r'^places$', generics.ListCreateAPIView.as_view(model=Place)),
    url(r'^places/(?P<pk>[^/]+)/$', generics.RetrieveAPIView.as_view(model=Place)),

    url(r'^robots$', PrivateView.as_view(model=Robot,serializer_class=RobotSerializer)),
    url(r'^robots/(?P<pk>[^/]+)/$', generics.RetrieveAPIView.as_view(model=Robot,serializer_class=RobotSerializer)),
)
