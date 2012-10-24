define([
    'ember',
    'app'
], function( Ember, App) {

  App.LookController = Ember.ObjectController.extend({
    pointHeadForward: function() {
      this.get('content')._pointHeadForward();
    },

  });

});

