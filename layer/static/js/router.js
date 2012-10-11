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

        /* Initialize the "navigate" state */
        connectOutlets: function(router, context) {
          /* Set the ApplicationView's {{outlet}} to be a RobotView with
           * a RobotController which has a Robot model as context */
          router.get('applicationController')
            .connectOutlet('robot', App.Robot.find(context.id));
          /* Set the RobotView's {{outlet}} to be a MapView with a
             MapController using a Place model as context */
          router.get('navigateController')
            .connectOutlet('map', App.Place.find({format:'json'}));
          router.get('robotController')
            .connectOutlet('navigate', App.Robot.find(context.id));
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

        connectOutlets: function(router, context) {
          robot = App.Robot.find(context.robot_id);
          place = App.Place.find(context.place_id);
          router.get('applicationController')
            .connectOutlet('navigating', Ember.Object.create({
              robot: robot,
              place: place
            }));
          robot.navigateTo(place);
        }
      })
    })
  });

});
