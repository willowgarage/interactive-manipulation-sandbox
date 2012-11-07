define([
  'ember',
  'app',
  'text!templates/plug.handlebars'
], function( Ember, App, plugHtml) {

    App.PlugView = Ember.View.extend({
        template: Ember.Handlebars.compile(plugHtml),
        pluggedInStatusChanged: function() {
          $('.mybutton').removeClass('disabled');
        }.observes('controller.content.plugged_in')
    });

});
