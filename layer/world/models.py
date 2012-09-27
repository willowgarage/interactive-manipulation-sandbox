from django.db import models

class Place(model.Model):
    name = model.TextField()
    tags = model.ListField()
    image = model.ImageField()

    pose_x = model.FloatField()
    pose_y = model.FloatField()
    pose_angle = model.FloatField()

    map_x = model.IntegerField()
    map_y = model.integerField()
    map_width = model.IntegerField()
    map_height = model.IntegerField()
