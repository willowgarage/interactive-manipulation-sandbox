from django.contrib import admin
from world.models import Place, Robot

class RobotAdmin(admin.ModelAdmin):
    list_display = ('name', 'description')

class PlaceAdmin(admin.ModelAdmin):
    list_display = ('name', 'description', 'pose_x', 'pose_y', 'map_x', 'map_y')

admin.site.register(Place, PlaceAdmin)
admin.site.register(Robot, RobotAdmin)
