define([
  'ember',
  'app',
  'text!templates/client.handlebars'
], function( Ember, App, clientHtml) {

    App.ClientView = Ember.View.extend({
        template: Ember.Handlebars.compile(clientHtml)
    });

});
