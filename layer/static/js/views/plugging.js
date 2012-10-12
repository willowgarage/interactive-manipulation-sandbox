define([
  'ember',
  'app',
  'text!templates/plugging.handlebars'
],
function(
  Ember,
  App,
  pluggingHtml
) {

  App.PluggingView = Ember.View.extend({
    template: Ember.Handlebars.compile(pluggingHtml)
  });

});

