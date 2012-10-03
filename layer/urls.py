from django.conf.urls.defaults import patterns, include, url

# Uncomment the next two lines to enable the admin:
from django.contrib import admin
admin.autodiscover()

urlpatterns = patterns('',
    # Examples:
    # url(r'^$', 'layer.views.home', name='home'),
    # url(r'^layer/', include('layer.foo.urls')),

    # Uncomment the admin/doc line below to enable admin documentation:
    # url(r'^admin/doc/', include('django.contrib.admindocs.urls')),

    # Uncomment the next line to enable the admin:
    url(r'^admin/', include(admin.site.urls)),

    # REST framework Login/Logout views (see http://django-rest-framework.org/)
    url(r'^api-auth/', include('djangorestframework.urls', namespace='djangorestframework')),

    # JAC: layer application URLs
    # Robot Manager
    url(r'^robotman/$', 'layer.robotman.views.home'),

    # Database of objects and prototype application
    url(r'^world/', include('layer.world.urls')),
)
