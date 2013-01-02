define([
  'ember',
  'app',
  'text!templates/robot.handlebars'
], function( Ember, App, robotHtml) {

    App.RobotView = Ember.View.extend({
        template: Ember.Handlebars.compile(robotHtml)
    });

});
