from django.conf.urls.defaults import patterns, include, url

urlpatterns = patterns('',
    # HACK: Keeping track of connected users
    url(r'^api/client', 'world.views.context'),

    # Separate REST URL configuration on its own file
    url(r'^api/', include('world.resturls')),
)
