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
    look_cameras = CameraSerializer(source='look_cameras')
    class Meta:
        model = Robot

    def get_related_field(self, model_field, to_many):
        # HACK: This assumes the only related field in the Robot class
        # is always a camera. We need to fix this if we want to add any
        # other relation in the Robot class
        return CameraSerializer()

urlpatterns = patterns('',
    url(r'^places$', generics.ListCreateAPIView.as_view(model=Place)),
    url(r'^places/(?P<pk>[^/]+)/$', generics.RetrieveAPIView.as_view(model=Place)),

    url(r'^robots$', PrivateView.as_view(model=Robot,serializer_class=RobotSerializer)),
    url(r'^robots/(?P<pk>[^/]+)/$', generics.RetrieveAPIView.as_view(model=Robot,serializer_class=RobotSerializer)),
)
