# Django settings for layer project.

# NOTE: The settings you configure here are going to be deployed on babylon1
# If you want to change a setting in your own local development environment
# or on a test server you're running, please create a file named
# local_settings.py in the layer/ directory and override any settings you
# need in that file

DEBUG = True
TEMPLATE_DEBUG = DEBUG

ADMINS = (
    # ('Your Name', 'your_email@example.com'),
)

MANAGERS = ADMINS

DATABASES = {
    'default': {
#        'ENGINE': 'django.db.backends.',  # Add 'postgresql_psycopg2', 'postgresql', 'mysql', 'sqlite3' or 'oracle'.
#        'NAME': '',                       # Or path to database file if using sqlite3.
        'ENGINE': 'django_mongodb_engine', # Add 'postgresql_psycopg2', 'postgresql', 'mysql', 'sqlite3' or 'oracle'.
        'NAME': 'rwt',                     # Or path to database file if using sqlite3.
        'USER': '',                      # Not used with sqlite3.
        'PASSWORD': '',                  # Not used with sqlite3.
        'HOST': 'localhost',                      # Set to empty string for localhost. Not used with sqlite3.
        'PORT': '',                      # Set to empty string for default. Not used with sqlite3.
    }
}

# Local time zone for this installation. Choices can be found here:
# http://en.wikipedia.org/wiki/List_of_tz_zones_by_name
# although not all choices may be available on all operating systems.
# On Unix systems, a value of None will cause Django to use the same
# timezone as the operating system.
# If running in a Windows environment this must be set to the same as your
# system time zone.
TIME_ZONE = 'America/Los_Angeles'

# Language code for this installation. All choices can be found here:
# http://www.i18nguy.com/unicode/language-identifiers.html
LANGUAGE_CODE = 'en-us'

SITE_ID = "506384d49932933347a03820"

# If you set this to False, Django will make some optimizations so as not
# to load the internationalization machinery.
USE_I18N = True

# If you set this to False, Django will not format dates, numbers and
# calendars according to the current locale
USE_L10N = True

# Absolute filesystem path to the directory that will hold user-uploaded files.
# Example: "/home/media/media.lawrence.com/media/"
MEDIA_ROOT = ''

# URL that handles the media served from MEDIA_ROOT. Make sure to use a
# trailing slash.
# Examples: "http://media.lawrence.com/media/", "http://example.com/media/"
MEDIA_URL = '/'

# Absolute path to the directory static files should be collected to.
# Don't put anything in this directory yourself; store your static files
# in apps' "static/" subdirectories and in STATICFILES_DIRS.
# Example: "/home/media/media.lawrence.com/static/"
STATIC_ROOT = ''

# URL prefix for static files.
# Example: "http://media.lawrence.com/static/"
STATIC_URL = '/static/'

# URL prefix for admin static files -- CSS, JavaScript and images.
# Make sure to use a trailing slash.
# Examples: "http://foo.com/static/admin/", "/static/admin/".
ADMIN_MEDIA_PREFIX = '/static/admin/'

# Additional locations of static files
STATICFILES_DIRS = (
    # Put strings here, like "/home/html/static" or "C:/www/django/static".
    # Always use forward slashes, even on Windows.
    # Don't forget to use absolute paths, not relative paths.
    "/home/rwt/layer/static/",
)

# List of finder classes that know how to find static files in
# various locations.
STATICFILES_FINDERS = (
    'django.contrib.staticfiles.finders.FileSystemFinder',
    'django.contrib.staticfiles.finders.AppDirectoriesFinder',
#    'django.contrib.staticfiles.finders.DefaultStorageFinder',
)

# Make this unique, and don't share it with anybody.
SECRET_KEY = '+9qt4^w8h4ah2n$$a2&gk59g^b(ydtr&%l+a%)8#e6q9sr98xe'

# List of callables that know how to import templates from various sources.
TEMPLATE_LOADERS = (
    'django.template.loaders.filesystem.Loader',
    'django.template.loaders.app_directories.Loader',
#     'django.template.loaders.eggs.Loader',
)

MIDDLEWARE_CLASSES = (
    'django.middleware.common.CommonMiddleware',
    'django.contrib.sessions.middleware.SessionMiddleware',
    'django.middleware.csrf.CsrfViewMiddleware',
    'django.contrib.auth.middleware.AuthenticationMiddleware',
    'django.contrib.messages.middleware.MessageMiddleware',
)

ROOT_URLCONF = 'layer.urls'

TEMPLATE_DIRS = (
    # Put strings here, like "/home/html/django_templates" or "C:/www/django/templates".
    # Always use forward slashes, even on Windows.
    # Don't forget to use absolute paths, not relative paths.
    '/home/rwt/layer/templates'
)

AUTHENTICATION_BACKENDS = (
    'layer.auth.HackedOpenIDBackend',
    'django.contrib.auth.backends.ModelBackend',
)


LOGIN_URL = '/accounts/login/'
LOGIN_REDIRECT_URL = '/'
OPENID_UPDATE_DETAILS_FROM_SREG = True
OPENID_CREATE_USERS = True
OPENID_SSO_SERVER_URL = 'https://www.google.com/accounts/o8/id'
# The actual URL for authentication using Google Apps (Willow Garage Domain, not Google or any other)
# is the following one:
# OPENID_SSO_SERVER_URL = 'https://www.google.com/accounts/o8/site-xrds?hd=willowgarage.com'
# but it fails in the underlying python-openid layer with:
# 
#   Error attempting to use stored discovery information: <openid.consumer.consumer.TypeURIMismatch: Required type http://specs.openid.net/auth/2.0/signon not found in ['http://specs.openid.net/auth/2.0/server', 'http://openid.net/srv/ax/1.0', 'http://specs.openid.net/extensions/ui/1.0/mode/popup', 'http://specs.openid.net/extensions/ui/1.0/icon', 'http://specs.openid.net/extensions/pape/1.0'] for endpoint <openid.consumer.discover.OpenIDServiceEndpoint server_url='https://www.google.com/a/willowgarage.com/o8/ud?be=o8' claimed_id=None local_id=None canonicalID=None used_yadis=True >>
#   Attempting discovery to verify endpoint
#   Performing discovery on http://willowgarage.com/openid?id=117302733910584657424
#
# and then it times out
#

INSTALLED_APPS = (
    'django.contrib.auth',
    'django.contrib.contenttypes',
    'django.contrib.sessions',
    'django.contrib.sites',
    'django.contrib.messages',
    'django.contrib.staticfiles',
    # Uncomment the next line to enable the admin:
    'django.contrib.admin',
    # Uncomment the next line to enable admin documentation:
    # 'django.contrib.admindocs',
    'django_mongodb_engine',

    # Other dependencies
    'djangorestframework',
    'django_openid_auth',
    'ember',

    # Layer applications
    'layer.world',
    'layer.robotman',
)

# A sample logging configuration. The only tangible logging
# performed by this configuration is to send an email to
# the site admins on every HTTP 500 error.
# See http://docs.djangoproject.com/en/dev/topics/logging for
# more details on how to customize your logging configuration.
LOGGING = {
    'version': 1,
    'disable_existing_loggers': False,
    'handlers': {
        'mail_admins': {
            'level': 'ERROR',
            'class': 'django.utils.log.AdminEmailHandler'
        }
    },
    'loggers': {
        'django.request': {
            'handlers': ['mail_admins'],
            'level': 'ERROR',
            'propagate': True,
        },
    }
}

from django.template import add_to_builtins
add_to_builtins('ember.templatetags.ember')

try:
    from layer.local_settings import *
    print "Using settings from local_settings.py"
except ImportError:
    print "Local settings not found.\nNOTE: If you want to override django settings for this particular computer, please create a local_settings.py file in your root django directory and perform any desired configuration there"
