define([
  'ember',
  'app',
  'text!templates/move.handlebars'
], function( Ember, App, moveHtml) {

    var registerButton = function ($button, robot, startMovingCallback, stopMovingCallback) {
      $button.on('mousedown', function (event) {
        event.preventDefault();
        startMovingCallback.apply(robot);;
        $button.on('mouseup mouseout', function (event) {
          stopMovingCallback.apply(robot);
          $button.off('mouseup mouseout');
        });
      });
    };

    App.MoveView = Ember.View.extend({
      template: Ember.Handlebars.compile(moveHtml),

      didInsertElement: function () {
        var controller = this.get('controller')
          , robot = this.get('context')
          , bindButton = function (selector, motionName) {
              return registerButton(
                Ember.$(selector),
                robot,
                controller.get(motionName),
                controller.get('stopMoving')
              );
          };

        bindButton("#move-forward", "moveForward");
        bindButton("#move-back", 'moveBack');
        bindButton("#turn-left", 'turnLeft');
        bindButton("#turn-right", 'turnRight');
      }
    });

});
