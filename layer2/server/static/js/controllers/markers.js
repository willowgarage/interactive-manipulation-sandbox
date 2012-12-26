define([
    'ember',
    'app'
], function( Ember, App) {

  App.MarkersController = Ember.ObjectController.extend({
    grasp: function() {
      var robot = this.get('content');
      robot.interactiveGripper('grasp', 'right', true);
    },

    move: function() {
      var robot = this.get('content');
      robot.interactiveGripper('move', 'right', false);
    },

    place: function() {
      var robot = this.get('content');
      robot.interactiveGripper('place', 'right', true);
    },

    segmentAndRecognize: function() {
      var robot = this.get('content');
      robot.segmentAndRecognize();
    }

  });

});

