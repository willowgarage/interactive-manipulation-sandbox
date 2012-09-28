requirejs.config({
  enforceDefine: true

, paths: {
    // Libraries
    jquery        : 'https://ajax.googleapis.com/ajax/libs/jquery/1.8.2/jquery.min'
  , underscore    : 'http://underscorejs.org/underscore'
  , backbone      : 'http://backbonejs.org/backbone'
    // Require.js Plugins
  , text          : 'https://raw.github.com/requirejs/text/latest/text'
    // ROS
  , eventemitter2 : 'https://raw.github.com/hij1nx/EventEmitter2/master/lib/eventemitter2'
  , ROS           : 'https://raw.github.com/RobotWebTools/rosjs/fuerte-devel/ros'
    // Templates
  , templates     : '../templates'
  }

, shim: {
    'underscore': {
      exports: '_'
    }
  , 'backbone': {
      deps    : ['underscore', 'jquery']
    , exports : 'Backbone'
    }
  }
});

define(['app'], function(App) {

});

