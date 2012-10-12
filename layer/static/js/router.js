define([
  'ember',
  'app',
  'controllers/application',
  'controllers/navigating',
  'controllers/robots',
  'controllers/map',
  'controllers/robot',
  'controllers/plug',
  'controllers/navigate',
  'views/application',
  'views/navigating',
  'views/robots',
  'views/map',
  'views/robot',
  'views/plug',
  'views/navigate',
  'models/robot',
  'models/place'
],
function(
  Ember,
  App,
  ApplicationController,
  NavigatingController,
  RobotsController,
  MapController,
  RobotController,
  PlugController,
  NavigateController,
  ApplicationView,
  NavigatingView,
  RobotsView,
  RobotView,
  PlugView,
  NavigateView,
  Robot,
  Place
) {

  App.Router = Ember.Router.extend({
    root : Ember.Route.extend({
      index: Ember.Route.extend({
        route: '/',

        redirectsTo: 'robots'
      }),

      robots: Ember.Route.extend({
        route: '/robots',

        showRobot: Ember.Route.transitionTo('navigate'),

        connectOutlets: function(router) {
          router.get('applicationController')
            .connectOutlet('robots', App.Robot.find({format:'json'}));
        }
      }),

      robot: Ember.Route.extend({
        route: '/robots/:id',

        redirectsTo: 'navigate',

        showAllRobots: Ember.Route.transitionTo('robots')
      }),

      navigate : Ember.Route.extend({
        route: '/robots/:id/navigate',

        plug: Ember.Route.transitionTo('plug'),

        navigateTo: Ember.Route.transitionTo('navigating'),
        showAllRobots: Ember.Route.transitionTo('robots'),

        unplug: function() {
          this.robot.unplug();
        },

        /* Initialize the "navigate" state */
        connectOutlets: function(router, context) {
          this.robot = App.Robot.find(context.id);
          /* Set the ApplicationView's {{outlet}} to be a RobotView with
           * a RobotController which has a Robot model as context */
          router.get('applicationController')
            .connectOutlet('robot', this.robot);
          /* Set the RobotView's {{outlet}} to be a NavigateView with
           * a NagivateController which has a Robot model as context */
          router.get('robotController')
            .connectOutlet('navigate', this.robot);
          /* Set the NagivateView's {{outlet}} to be a MapView with a
             MapController using a Place model as context */
          router.get('navigateController')
            .connectOutlet('map', App.Place.find({format:'json'}));
        }
      }),

      plug : Ember.Route.extend({
        route: '/robots/:id/plug',

        navigate: Ember.Route.transitionTo('navigate'),
        showAllRobots: Ember.Route.transitionTo('robots'),

        connectOutlets: function(router, context) {
          router.get('applicationController')
            .connectOutlet('robot', App.Robot.find(context.id));
          router.get('robotController')
            .connectOutlet('plug', App.Robot.find(context.id));
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
