requirejs.config({
  enforceDefine: true,
  waitSeconds: 300,

  paths: {
    jquery       : 'libs/jquery',
    blockUI      : 'libs/jquery.blockUI',
    d3           : 'libs/d3',
    // Ember
    handlebars   : 'libs/handlebars',
    ember        : 'libs/ember',
    emberdata    : 'libs/ember-data',
    // Require.js Plugins
    text         : 'libs/text',
    // ROS
    eventemitter2 : 'libs/eventemitter2',
    ROS           : 'libs/ros',
    actionclient  : 'libs/actionclient',
    action        : 'libs/action',
    // socket.io
    socketio      : 'libs/socket.io',
    // Templates
    templates     : '../templates'
  },

  shim: {
    'jquery': {
      deps: ['blockUI'],
    },
    'socketio': {
      exports: 'socketio'
    },
    'd3': {
      exports: 'd3'
    },
    'handlebars': {
      exports: 'Handlebars'
    },
    'ember': {
      deps    : ['jquery', 'handlebars'],
      exports : 'Ember'
    },
    'emberdata': {
      deps    : ['jquery', 'ember'],
      exports : 'DS'
    }
  }
});

define([
  'app',
  'router'
],
function( App, Router) {
  $(".loading").remove();
  App.initialize();
});
