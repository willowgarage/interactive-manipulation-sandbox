define([
  'ember',
  'app',
  'text!templates/markers.handlebars'
], function( Ember, App, markersHtml) {

    App.MarkersView = Ember.View.extend({
        template: Ember.Handlebars.compile(markersHtml)
    });

});
