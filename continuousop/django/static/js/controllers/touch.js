define([
  'ember',
  'app',
  'jquery'
], function(Ember, App, $) {

  App.TouchController = Ember.ObjectController.extend({

    current_state: 'move',

    pointHeadClick: function(arg) {
      // Workaround for Firefox because it does not provide the offsetX/Y values
      if (typeof arg.offsetX === 'undefined' || typeof arg.offsetY === 'undefined') {
        var targetOffset = $(arg.target).offset();
        arg.offsetX = arg.pageX - targetOffset.left;
        arg.offsetY = arg.pageY - targetOffset.top;
      }

      var fracX = arg.offsetX / arg.target.clientWidth;
      var fracY = arg.offsetY / arg.target.clientHeight;
      this.get('content').pointHeadClick(fracX, fracY);
    },

    setState: function(arg) {
      // Style the tabs so the newly selected one is highlighted
      $('.current').removeClass('current');
      $(arg.currentTarget).addClass('current');

      // Display the selected tab's content in the page
      var newState = arg.currentTarget.attributes.param.value;
      var camera = arg.currentTarget.attributes.camera.value;
      this.set('current_state', newState);
      this.connectOutlet(newState, this.get('content'));

      // Update the selected camera to display the right one for this tab (doesn't work)
      var robot = this.get('content');
      var cameraObject = robot.getCameraByName(camera);
      console.log("Got camera object:", cameraObject.name);
      robot.set('selected_camera', cameraObject);
    }

  });

});

