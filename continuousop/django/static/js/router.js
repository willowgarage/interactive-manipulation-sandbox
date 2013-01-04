define([
  'ember',
  'app',
  'controllers/application',
  'controllers/navigating',
  'controllers/robots',
  'controllers/map',
  'controllers/robot',
  'controllers/move',
  'controllers/plug',
  'controllers/navigate',
  'controllers/touch',
  'controllers/client',
  'controllers/markers',
  'controllers/pickup',
  'views/application',
  'views/navigating',
  'views/robots',
  'views/map',
  'views/robot',
  'views/move',
  'views/plug',
  'views/navigate',
  'views/touch',
  'views/client',
  'views/markers',
  'views/pickup',
  'models/robot',
  'models/place',
  'models/client',
  'helpers/all'
],
function(
  Ember,
  App,
  ApplicationController,
  NavigatingController,
  RobotsController,
  MapController,
  RobotController,
  LookController,
  PlugController,
  NavigateController,
  LookController,
  ClientController,
  MarkersController,
  PickupController,
  ApplicationView,
  NavigatingView,
  RobotsView,
  MapView,
  RobotView,
  LookView,
  PlugView,
  NavigateView,
  LookView,
  ClientView,
  MarkersView,
  PickupView,
  Robot,
  Place,
  Client,
  Helpers
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
          //  New HACK for synchronizing current UI view.
          //  This needs to be done in connectOutlet for each route
          //  until I can figure out how to add an observer for this
          App.setClientContext('robots');

          router.get('applicationController')
            .connectOutlet('content', 'robots', App.Robot.find({format:'json'}));
          router.get('robotsController')
            .connectOutlet('client', 'client', App.client);
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

        touch: Ember.Route.transitionTo('touch'),
        markers: Ember.Route.transitionTo('markers'),

        // TODO: Clean-up
        navigateTo: function( router, context) {
          var navigatingContext = {
            place_id: router.get('mapController').get('placeId'),
            robot_id: router.get('robotController').get('content').get('id')
          };
          Ember.Route.transitionTo('navigating')(router,navigatingContext);
        },

        /* Initialize the 'navigate' state */
        connectOutlets: function(router, context) {
          //  New HACK for synchronizing current UI view.
          //  This needs to be done in connectOutlet for each route
          //  until I can figure out how to add an observer for this
          App.setClientContext('robot:'+context.id);

          this.robot =  App.Robot.find(context.id);
          // JAC: NOTE I'm using connectOutlet(<outlet name>,<view>,<context>)
          /* Set the ApplicationView's {{outlet}} to be a RobotView with
           * a RobotController which has a Robot model as context */
          router.get('applicationController').
            connectOutlet('content','robot', this.robot);
          /* And also update the client status information view with the new
           * application context (i.e.: where the user is now in the app) */
          router.get('robotController').
            connectOutlet('periphery','client',App.client);
          /* Set the RobotView's {{outlet}} to be a NavigateView with
           * a NagivateController which has a Robot model as context */
          router.get('robotController')
            .connectOutlet('main', 'navigate', this.robot);
          /* Set the NagivateView's {{outlet}} to be a MapView with a
             MapController using a Place model as context */
          router.get('navigateController')
            .connectOutlet('map', Ember.Object.create({
              enablePlaces: true,
              enableRobot: true,
              enableDestination: false,
              robot: this.robot,
              places: App.Place.find({format:'json'})
            }));
        }
      }),

      touch: Ember.Route.extend({
        route: '/robots/:id/touch',
        showAllRobots: Ember.Route.transitionTo('robots'),
        navigate: Ember.Route.transitionTo('navigate'),
        markers: Ember.Route.transitionTo('markers'),

        connectOutlets: function(router, context) {
          //  New HACK for synchronizing current UI view.
          //  This needs to be done in connectOutlet for each route
          //  until I can figure out how to add an observer for this
          App.setClientContext('robot:'+context.id);

          var robot = App.Robot.find(context.id);
          router.get('applicationController').
            connectOutlet('content','robot', robot);
          router.get('robotController').
            connectOutlet('periphery','client', App.client);
          router.get('robotController')
            .connectOutlet('main', 'touch', robot);
          router.get('touchController')
            .connectOutlet('move', robot);
        }
      }),

      navigating: Ember.Route.extend({
        route: '/robots/:robot_id/navigating/:place_id',
        showAllRobots: Ember.Route.transitionTo('robots'),
        navigate: Ember.Route.transitionTo('navigate'),
        touch: Ember.Route.transitionTo('touch'),
        markers: Ember.Route.transitionTo('markers'),

        cancelAllGoals: function(router) {
          this.robot.cancelAllGoals();
        },

        connectOutlets: function(router, context) {
          //  New HACK for synchronizing current UI view.
          //  This needs to be done in connectOutlet for each route
          //  until I can figure out how to add an observer for this
          App.setClientContext('robot:'+context.robot_id);

          this.robot = App.Robot.find(context.robot_id);
          this.place = App.Place.find(context.place_id);
          router.get('applicationController').
            connectOutlet('content','robot',this.robot);
          router.get('robotController').
            connectOutlet('periphery','client', App.client);
          router.get('robotController').
            connectOutlet('main', 'navigating', Ember.Object.create({
              robot: this.robot,
              place: this.place
            }));
          router.get('navigatingController').
            connectOutlet('map', Ember.Object.create({
              enablePlaces: false,
              enableRobot: true,
              enableDestination: true,
              robot: this.robot,
              place: this.place,
              places: App.Place.find({format:'json'})
            }));
          // Send the robot the actual navigateTo command via ROS
          this.robot.navigateTo(this.place);
        }
      }),

      markers: Ember.Route.extend({
        route: '/robots/:id/markers',
        showAllRobots: Ember.Route.transitionTo('robots'),
        navigate: Ember.Route.transitionTo('navigate'),
        touch: Ember.Route.transitionTo('touch'),

        connectOutlets: function(router, context) {
          //  New HACK for synchronizing current UI view.
          //  This needs to be done in connectOutlet for each route
          //  until I can figure out how to add an observer for this
          App.setClientContext('robot:'+context.robot_id);

          router.get('applicationController')
            .connectOutlet('content', 'robot', App.Robot.find(context.id));
          router.get('robotController').
            connectOutlet('periphery', 'client', App.client);
          router.get('robotController')
            .connectOutlet('main', 'markers', App.Robot.find(context.id));
        }
      })

    })
  });
});

