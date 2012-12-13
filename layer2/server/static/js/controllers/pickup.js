define([
    'ember',
    'app'
], function( Ember, App) {

  App.PickupController = Ember.ObjectController.extend({

    found_objects: [],

    segmentAndRecognize: function() {
      var robot = this.get('content');
      robot.segmentAndRecognize();
    }

  });

});

