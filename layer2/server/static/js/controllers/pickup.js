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
        alert("No object selected!");
        return;
      }

      alert("Pickup not implemented yet, selected object: " + this.selected_object);
    }

  });

});

