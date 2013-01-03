define([
    'ember',
    'app'
], function( Ember, App) {

  App.PickupController = Ember.ObjectController.extend({

    found_objects: [],
    selected_object: null,

    segmentAndRecognize: function() {
      var robot = this.get('content');
      robot.segmentAndRecognize();
    },

    pickupObject: function() {
      if (this.selected_object === null) {
        alert('No object selected!');
        return;
      }

      var robot = this.get('content');
      robot.pickupObject(this.selected_object);
    },

    untuckArms: function() {
      var robot = this.get('content');
      robot.untuckArmsForGrasping(this.selected_object);
    }

  });

});

