define([
    'ember',
    'app'
], function( Ember, App) {

  App.MarkersController = Ember.ObjectController.extend({
    grasp: function() {
      var robot = this.get('content');
      robot.interactive_gripper('grasp', 'right', true);
    },

    move: function() {
      var robot = this.get('content');
      robot.interactive_gripper('move', 'right', false);
    },

    place: function() {
      var robot = this.get('content');
      robot.interactive_gripper('place', 'right', true);
    }

  });

});

