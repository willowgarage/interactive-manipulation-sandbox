# -*- coding: utf-8 -*-
import datetime
from south.db import db
from south.v2 import DataMigration
from django.db import models

class Migration(DataMigration):
    """Non-reversible migration to remove US places, and
    replaces them with Argentina's."""

    def forwards(self, orm):
        """Remove US places, add Argentina places."""
        # Remove all current bin locations, and bins.
        for place in orm['world.Place'].objects.all():
            place.delete()

        orm['world.Place'](name="Bathroom",
                           pose_x=1.368, pose_y=-4.20446, pose_angle=-3.0579,
                           map_x=228, map_y=290, map_width=24, map_height=26
                           ).save()
        orm['world.Place'](name="Kitchen",
                           pose_x=-1.817, pose_y=-3.641, pose_angle=-3.0579,
                           map_x=54, map_y=277, map_width=24, map_height=26
                           ).save()
        orm['world.Place'](name="Plug",
                           pose_x=-0.295, pose_y=2.333, pose_angle=1.2545,
                           map_x=97, map_y=35, map_width=20, map_height=20
                           ).save()
        orm['world.Place'](name="Mariano's desk",
                           pose_x=1.333, pose_y=2.156, pose_angle=-1.595,
                           map_x=173, map_y=36, map_width=34, map_height=38
                           ).save()
        orm['world.Place'](name="Tessa's desk",
                           pose_x=-1.44, pose_y=-0.693, pose_angle=-1.5206,
                           map_x=97, map_y=176, map_width=25, map_height=25
                           ).save()
        orm['world.Place'](name="Julian's desk",
                           pose_x=-2.155, pose_y=-0.377, pose_angle=2.043,
                           map_x=38, map_y=170, map_width=20, map_height=20
                           ).save()
        orm['world.Place'](name="Elvio's desk",
                           pose_x=-1.299, pose_y=2.4532, pose_angle=-2.253,
                           map_x=55, map_y=30, map_width=25, map_height=25
                           ).save()
        orm['world.Place'](name="Meeting room",
                           pose_x=2.406, pose_y=-2.209, pose_angle=1.154,
                           map_x=251, map_y=226, map_width=25, map_height=25
                           ).save()

    def backwards(self, orm):
        raise NotImplementedError()

    models = {
        'world.camera': {
            'Meta': {'object_name': 'Camera'},
            'features': ('django.db.models.fields.CharField', [], {'max_length': '512', 'blank': 'True'}),
            'id': ('django.db.models.fields.AutoField', [], {'primary_key': 'True'}),
            'name': ('django.db.models.fields.CharField', [], {'max_length': '64'}),
            'url': ('django.db.models.fields.CharField', [], {'max_length': '512'})
        },
        'world.place': {
            'Meta': {'object_name': 'Place'},
            'description': ('django.db.models.fields.TextField', [], {'null': 'True', 'blank': 'True'}),
            'id': ('django.db.models.fields.AutoField', [], {'primary_key': 'True'}),
            'image': ('django.db.models.fields.files.ImageField', [], {'max_length': '100', 'null': 'True', 'blank': 'True'}),
            'map_height': ('django.db.models.fields.IntegerField', [], {'null': 'True', 'blank': 'True'}),
            'map_width': ('django.db.models.fields.IntegerField', [], {'null': 'True', 'blank': 'True'}),
            'map_x': ('django.db.models.fields.IntegerField', [], {'null': 'True', 'blank': 'True'}),
            'map_y': ('django.db.models.fields.IntegerField', [], {'null': 'True', 'blank': 'True'}),
            'name': ('django.db.models.fields.TextField', [], {}),
            'pose_angle': ('django.db.models.fields.FloatField', [], {'null': 'True', 'blank': 'True'}),
            'pose_x': ('django.db.models.fields.FloatField', [], {'null': 'True', 'blank': 'True'}),
            'pose_y': ('django.db.models.fields.FloatField', [], {'null': 'True', 'blank': 'True'}),
            'tags': ('django.db.models.fields.TextField', [], {'null': 'True', 'blank': 'True'})
        },
        'world.robot': {
            'Meta': {'object_name': 'Robot'},
            'camera_base_url': ('django.db.models.fields.CharField', [], {'max_length': '128'}),
            'cameras': ('django.db.models.fields.related.ManyToManyField', [], {'to': "orm['world.Camera']", 'symmetrical': 'False'}),
            'description': ('django.db.models.fields.TextField', [], {'null': 'True', 'blank': 'True'}),
            'id': ('django.db.models.fields.AutoField', [], {'primary_key': 'True'}),
            'image': ('django.db.models.fields.files.ImageField', [], {'max_length': '100', 'null': 'True', 'blank': 'True'}),
            'name': ('django.db.models.fields.TextField', [], {}),
            'public': ('django.db.models.fields.NullBooleanField', [], {'null': 'True', 'blank': 'True'}),
            'service_url': ('django.db.models.fields.CharField', [], {'max_length': '512'}),
            'state': ('django.db.models.fields.IntegerField', [], {}),
            'tags': ('django.db.models.fields.TextField', [], {'null': 'True', 'blank': 'True'})
        }
    }

    complete_apps = ['world']
    symmetrical = True
