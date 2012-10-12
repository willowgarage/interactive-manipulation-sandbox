define([
  'ember',
  'app',
  'text!templates/robot.handlebars'
], function( Ember, App, robotHtml) {

    App.RobotView = Ember.View.extend({
        template: Ember.Handlebars.compile(robotHtml),
        didInsertElement: function() {
          $('#login-logout-link').insertBefore(".robot_info")
            .css('float','right')
            .css('margin-right','20px')
            .css('margin-left','20px');
        },
        willDestroyElement: function() {
          $('#login-logout-link').appendTo("body");
        }
    });

});
