define([
  'ember',
  'app',
  'jquery'
], function(Ember, App, $) {

  App.MoveController = Ember.ObjectController.extend({
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
    }

  });

});

