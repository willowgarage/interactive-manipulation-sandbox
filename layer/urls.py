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

    # OpenID Login through Google:
    url(r'^accounts/', include('django_openid_auth.urls')),
    url(r'^accounts/logout', 'django.contrib.auth.views.logout', 
        {'template_name': 'index.html', 'next_page': '/'}),
    url('^$', 'django.views.generic.simple.direct_to_template',
        {'template': 'index.html'}),

    # Default Django Authentication
    #url(r'^accounts/login/$', 'django.contrib.auth.views.login'),
    #url(r'^accounts/logout/$', 'django.contrib.auth.views.logout'),

    # REST framework Login/Logout views (see http://django-rest-framework.org/)
    url(r'^api-auth/', include('djangorestframework.urls', namespace='djangorestframework')),

    # JAC: layer application URLs
    # Robot Manager
    url(r'^robotman/$', 'layer.robotman.views.home'),

    # Database of objects and prototype application
    url(r'^world/', include('layer.world.urls')),

    # TL: Robot access layer
    # url(r'^$', 'layer.world.views.index'),

)
