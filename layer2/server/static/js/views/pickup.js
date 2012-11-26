define([
  'ember',
  'app',
  'text!templates/pickup.handlebars'
], function( Ember, App, pickupHtml) {

    App.PickupView = Ember.View.extend({
        template: Ember.Handlebars.compile(pickupHtml)
    });

});
