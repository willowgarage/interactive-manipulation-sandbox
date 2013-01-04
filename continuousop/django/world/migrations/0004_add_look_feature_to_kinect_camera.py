# -*- coding: utf-8 -*-
import datetime
from south.db import db
from south.v2 import DataMigration
from django.db import models

class Migration(DataMigration):

    def forwards(self, orm):
        "Write your forwards methods here."
        # Note: Remember to use orm['appname.ModelName'] rather than "from appname.models..."
        for camera in orm['world.Camera'].objects.all():
            if camera.name == 'kinect':
                camera.features = 'look'
                camera.save()

    def backwards(self, orm):
        "Write your backwards methods here."
        for camera in orm['world.Camera'].objects.all():
            if camera.name == 'kinect':
                camera.features = ''
                camera.save()

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
