requirejs.config({
  enforceDefine: true,
  waitSeconds: 300,

  paths: {
    jquery       : 'libs/jquery',
    d3           : 'libs/d3',
    blockUI      : 'libs/jquery.blockUI',
    // Three
    three        : 'libs/interactivemarkersjs/examples/include/three',
    colladaloader: 'libs/interactivemarkersjs/examples/include/ColladaLoader',
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
    'd3': {
      exports: 'd3'
    },
    'three': {
      exports: 'THREE'
    },
    'handlebars': {
      exports: 'Handlebars'
    },
    'ember': {
      deps    : ['jquery', 'blockUI', 'handlebars'],
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
  'router',
  'jquery'
],
function(App, Router, $) {
  $('.loading').remove();
  App.initialize();
});
