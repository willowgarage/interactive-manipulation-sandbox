define([
  'ember',
  'app',
  'controllers/application',
  'controllers/navigating',
  'controllers/robots',
  'controllers/map',
  'controllers/robot',
  'views/application',
  'views/navigating',
  'views/robots',
  'views/map',
  'views/robot',
  'models/robot',
  'models/place'
],
function(
  Ember,
  App,
  ApplicationController,
  NavigatingController,
  RobotsController,
  RobotController,
  ApplicationView,
  NavigatingView,
  RobotsView,
  RobotView
) {

  App.Router = Ember.Router.extend({
    root : Ember.Route.extend({
      index: Ember.Route.extend({
        route: '/',

        redirectsTo: 'robots'
      }),

      robots: Ember.Route.extend({
        route: '/robots',

        showRobot: Ember.Route.transitionTo('robot'),

        connectOutlets: function(router) {
          router.get('applicationController')
            .connectOutlet('robots', App.Robot.find({format:'json'}));
        }
      }),

      robot: Ember.Route.extend({
        route: '/robots/:id',

        showAllRobots: Ember.Route.transitionTo('robots'),

        navigateTo: Ember.Route.transitionTo('navigating'),

        unplug: function() {
          this.robot.unplug();
        },

        connectOutlets: function(router, context) {
          this.robot = App.Robot.find(context.id);
          router.get('applicationController')
            .connectOutlet('robot', this.robot);
          router.get('robotController')
            .connectOutlet('map', App.Place.find({format:'json'}));
        }
      }),

      navigating: Ember.Route.extend({
        route: '/navigating/:robot_id/:place_id',

        showAllRobots: Ember.Route.transitionTo('robots'),

        plugIn: function() {
          this.robot.plugIn();
        },

        connectOutlets: function(router, context) {
          this.robot = App.Robot.find(context.robot_id);
          this.place = App.Place.find(context.place_id);
          router.get('applicationController')
            .connectOutlet('navigating', Ember.Object.create({
              robot: this.robot,
              place: this.place
            }));
          this.robot.navigateTo(this.place);
        }
      })
    })
  });
});
