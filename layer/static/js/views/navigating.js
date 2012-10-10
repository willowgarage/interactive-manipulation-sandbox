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

});

