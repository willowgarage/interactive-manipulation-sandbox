define([
  'ember',
  'app',
  'text!templates/touch.handlebars'
], function( Ember, App, touchHtml) {

    App.TouchView = Ember.View.extend({
      template: Ember.Handlebars.compile(touchHtml)

    });

});
