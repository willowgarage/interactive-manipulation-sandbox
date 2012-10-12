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
  'controllers/plugging',
  'views/application',
  'views/navigating',
  'views/robots',
  'views/map',
  'views/robot',
  'views/plug',
  'views/navigate',
  'views/plugging',
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
  PluggingController,
  ApplicationView,
  NavigatingView,
  RobotsView,
  RobotView,
  PlugView,
  NavigateView,
  PluggingView,
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
        showAllRobots: Ember.Route.transitionTo('robots'),

        plug: Ember.Route.transitionTo('plug'),

        // TODO: Clean-up
        navigateTo: function( router, context) {
          var navigatingContext = {
            place_id: router.get('mapController').get('placeId'),
            robot_id: router.get('robotController').get('content').get('id')
          };
          Ember.Route.transitionTo('navigating')(router,navigatingContext);
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
        showAllRobots: Ember.Route.transitionTo('robots'),

        navigate: Ember.Route.transitionTo('navigate'),

        plugIn: function( router, context) {
          var robot = context.context;
          robot.plugIn();
          Ember.Route.transitionTo('plugging')(router,robot);
        },
        unplug: function( router, context) {
          var robot = context.context;
          robot.unplug();
          Ember.Route.transitionTo('plugging')(router,robot);
        },

        connectOutlets: function(router, context) {
          router.get('applicationController')
            .connectOutlet('robot', App.Robot.find(context.id));
          router.get('robotController')
            .connectOutlet('plug', App.Robot.find(context.id));
        }
      }),

      /* View for displaying cameras during a plug/unplug behavior */
      plugging: Ember.Route.extend({
        route: '/robots/:id/plugging',
        showAllRobots: Ember.Route.transitionTo('robots'),

        navigate: Ember.Route.transitionTo('navigate'),

        connectOutlets: function(router, context) {
          var robot = App.Robot.find(context.id);
          router.get('applicationController')
            .connectOutlet('robot', robot);
          router.get('robotController')
            .connectOutlet('plugging', robot);
        }
      }),

      navigating: Ember.Route.extend({
        route: '/navigating/:robot_id/:place_id',
        showAllRobots: Ember.Route.transitionTo('robots'),
        plug: Ember.Route.transitionTo('plug'),
        navigate: Ember.Route.transitionTo('navigate'),

        connectOutlets: function(router, context) {
          this.robot = App.Robot.find(context.robot_id);
          this.place = App.Place.find(context.place_id);
          router.get('applicationController')
            .connectOutlet('robot', this.robot);
          router.get('robotController').
            connectOutlet('navigating', Ember.Object.create({
              robot: this.robot,
              place: this.place
            }));
          // Send the robot the actual navigateTo command via ROS
          this.robot.navigateTo(this.place);
        }
      })
    })
  });
});
