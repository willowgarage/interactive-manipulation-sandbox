"""
This class is only used for django-rest-framework in order to enable
read-only REST access to model objects
"""

from djangorestframework.mixins import ReadModelMixin
from djangorestframework.views import ModelView

class ReadOnlyModelView(ReadModelMixin, ModelView):
    """
    A view which provides default operations for viewing a single model object
    """
    _suffix = 'Instance'
