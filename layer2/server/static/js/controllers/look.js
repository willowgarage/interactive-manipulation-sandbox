define([
    'ember',
    'app'
], function( Ember, App) {

  App.LookController = Ember.ObjectController.extend({

    pointHeadClick: function(arg) {
      // Workaround for Firefox because it does not provide the offsetX/Y values
      if(typeof arg.offsetX === "undefined" || typeof arg.offsetY === "undefined") {
        var targetOffset = $(arg.target).offset();
        arg.offsetX = arg.pageX - targetOffset.left;
        arg.offsetY = arg.pageY - targetOffset.top;
      }

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

