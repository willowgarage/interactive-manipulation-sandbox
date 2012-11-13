define([
    'ember',
    'app'
], function( Ember, App) {

  App.LookController = Ember.ObjectController.extend({

    pointHeadClick: function(arg) {
      console.log("You clicked in the image at " + arg.offsetX + "," + arg.offsetY);
      console.dir(arg);
    },

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

    moveForward: function() {
      this.get('content').moveForward();
    },
    moveBack: function() {
      this.get('content').moveBack();
    },
    turnLeft: function() {
      this.get('content').turnLeft();
    },
    turnRight: function() {
      this.get('content').turnRight();
    },

  });

});

