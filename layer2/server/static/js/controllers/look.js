define([
    'ember',
    'app'
], function( Ember, App) {

  App.LookController = Ember.ObjectController.extend({

    pointHeadClick: function(arg) {
      var fracX = arg.offsetX / arg.target.clientWidth;
      var fracY = arg.offsetY / arg.target.clientHeight;
      this.get('content').pointHeadClick(fracX, fracY);
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

