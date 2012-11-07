define([
    'ember',
    'app'
], function( Ember, App) {

  App.LookController = Ember.ObjectController.extend({

    pointHeadForward: function() {
      this.get('content')._pointHeadForward();
    },

    pointHeadUp: function() {
      this.get('content').pointHead(1, 0, 0.1);
    },

    pointHeadDown: function() {
      this.get('content').pointHead(1, 0, -0.1);
    },

    pointHeadLeft: function() {
      this.get('content').pointHead(1, 0.1, 0);
    },

    pointHeadRight: function() {
      this.get('content').pointHead(1, -0.1, 0);
    },

  });

});

