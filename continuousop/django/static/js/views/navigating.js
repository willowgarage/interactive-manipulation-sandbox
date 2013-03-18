define([
  'ember',
  'app',
  'text!templates/navigating.handlebars'
],
function(
  Ember,
  App,
  navigatingHtml
) {

  App.NavigatingView = Ember.View.extend({
    template: Ember.Handlebars.compile(navigatingHtml)

  });

  Ember.Handlebars.registerHelper('showPlace', function(value) {
    // Show a place name appropriately:
    //   "plug" ==> "the plug"
    //   "Guy's desk" ==> "Guy's desk"

    if (value.trim().toLowerCase().indexOf("'s") === 0)
      return value;
    return "the " + value;
  });

});
