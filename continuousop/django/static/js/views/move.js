define([
  'ember',
  'app',
  'text!templates/move.handlebars'
], function( Ember, App, moveHtml) {

    App.MoveView = Ember.View.extend({
      template: Ember.Handlebars.compile(moveHtml)

    });

});
