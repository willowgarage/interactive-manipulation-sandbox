define([
  'ember',
  'app',
  'text!templates/navigate.handlebars'
], function( Ember, App, navigateHtml) {

    App.NavigateView = Ember.View.extend({
        template: Ember.Handlebars.compile(navigateHtml)
    });

});
