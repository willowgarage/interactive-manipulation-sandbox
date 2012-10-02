from django.db import models

# Create your models here.
class Robot(models.Model):
    hostname = models.TextField()
    name = models.TextField()
