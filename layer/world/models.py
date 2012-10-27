from django.db import models
from djangotoolbox.fields import ListField
from .forms import StringListField

"""
JAC: Commenting out TagsField: not used at the moment (giving trouble left and right)

class TagsField(ListField):
    def formfield(self, **kwargs):
        return models.Field.formfield(self, StringListField, **kwargs)
"""

class Place(models.Model):
    name = models.TextField()
    tags = models.TextField(blank=True,null=True)
    description = models.TextField(blank=True,null=True)
    image = models.ImageField(upload_to="static/images",blank=True,null=True)

    pose_x = models.FloatField(blank=True,null=True)
    pose_y = models.FloatField(blank=True,null=True)
    pose_angle = models.FloatField(blank=True,null=True)

    map_x = models.IntegerField(blank=True,null=True)
    map_y = models.IntegerField(blank=True,null=True)
    map_width = models.IntegerField(blank=True,null=True)
    map_height = models.IntegerField(blank=True,null=True)

class Robot(models.Model):
    name = models.TextField()
    tags = models.TextField(blank=True,null=True)
    description = models.TextField(blank=True,null=True)
    image = models.ImageField(upload_to="static/images",blank=True,null=True)

    public = models.NullBooleanField(blank=True,null=True)

    state = models.IntegerField(choices=(
        (0,'Idle'), (1,'Busy'),
    ))

    # Disabled using URLFields since for the time being they do not support
    # Web Socket addresses
    #service_url = models.URLField(verify_exists=False)     # URL to robot's rosbridge instance
    #camera_url = models.URLField(verify_exists=False)      # URL to mjpeg output for Robot's camera
    service_url = models.CharField(max_length=512)     # URL to robot's rosbridge instance
    camera_url = models.CharField(max_length=512)      # URL to mjpeg output for Robot's camera
    forearm_camera_url = models.CharField(max_length=512)      # URL to mjpeg output for Robot's forearm camera

class Client(models.Model):
    '''Represents a client (browser) connected to the server and it's state'''
    session_key = models.CharField(max_length=40)
    username = models.CharField(max_length=30)
    context = models.TextField()
