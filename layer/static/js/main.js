requirejs.config({
  enforceDefine: true

, paths: {
    jquery     : 'libs/jquery'
  , d3         : 'libs/d3'
    // Ember
  , handlebars : 'libs/handlebars'
  , ember      : 'libs/ember'
  , emberdata  : 'libs/ember-data'
    // Require.js Plugins
  , text       : 'libs/text'
    // Templates
  , templates  : '../templates'
  }

, shim: {
    'handlebars': {
      exports: 'Handlebars'
    }
  , 'ember': {
      deps    : ['jquery', 'handlebars']
    , exports : 'Ember'
    }
  }
});

define([
  'app'
, 'router'
],
function(
  App
, Router
) {
  App.initialize();
});

