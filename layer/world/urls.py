from django.conf.urls.defaults import patterns, include, url

urlpatterns = patterns('',
    url(r'^$', 'layer.world.views.index'),
    url(r'^api/client', 'layer.world.views.context'),

    # Separate REST URL configuration on its own file
    url(r'^api/', include('layer.world.resturls')),
)
