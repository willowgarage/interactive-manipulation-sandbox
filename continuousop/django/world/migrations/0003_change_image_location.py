# -*- coding: utf-8 -*-
import datetime
import re
import os.path
from south.db import db
from south.v2 import DataMigration
from django.db import models
from django.conf import settings

class Migration(DataMigration):

    """
    Both the Place and the Robot models have an optional "image"
    ImageField. Each of this fields has a relative path that needs to change.
    The change consists of removing any starting "static/" part from that image
    field.

    Updating the path to a FileField instance (which is the base for
    ImageField) is done by assigning to it a path relative to the
    MEDIA_ROOT. The image field itself is saved to the database as a path
    relative to MEDIA_ROOT, but is exposed here as a full path. Since the
    MEDIA_ROOT is part of the code I will apply the substitution directly on
    the image path.
    """

    def forwards(self, orm):
        target = settings.MEDIA_ROOT + 'static/images'
        target_re = re.compile(target)
        substitute = 'images'

        for model_name in ['world.Place', 'world.Robot']:
            for instance in orm[model_name].objects.all():
                # Images are optional.
                if instance.image:
                    new_path = re.sub(target_re, substitute, instance.image.path)
                    instance.image = new_path
                    instance.save()

    def backwards(self, orm):
        target = settings.MEDIA_ROOT + 'images'
        target_re = re.compile(target)
        substitute = 'static/images'

        for model_name in ['world.Place', 'world.Robot']:
            for instance in orm[model_name].objects.all():
                # Images are optional.
                if instance.image:
                    new_path = re.sub(target_re, substitute, instance.image.path)
                    instance.image = new_path
                    instance.save()

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
