from django.db import models
from djangotoolbox.fields import ListField

class Place(models.Model):
    name = models.TextField()
    tags = ListField()
    image = models.ImageField()

    pose_x = models.FloatField()
    pose_y = models.FloatField()
    pose_angle = models.FloatField()

    map_x = models.IntegerField()
    map_y = models.IntegerField()
    map_width = models.IntegerField()
    map_height = models.IntegerField()
