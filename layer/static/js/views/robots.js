define([
  'ember',
  'app',
  'text!templates/robots.handlebars'
],
function(
  Ember,
  App,
  robotsHtml
) {

  App.RobotsView = Ember.View.extend({
    template: Ember.Handlebars.compile(robotsHtml)
  });

});


