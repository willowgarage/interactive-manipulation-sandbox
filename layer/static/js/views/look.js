define([
  'ember',
  'app',
  'text!templates/look.handlebars'
], function( Ember, App, lookHtml) {

    App.LookView = Ember.View.extend({
        template: Ember.Handlebars.compile(lookHtml)
    });

});
