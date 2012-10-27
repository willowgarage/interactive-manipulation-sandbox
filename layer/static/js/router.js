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
  'controllers/look',
  'controllers/client',
  'views/application',
  'views/navigating',
  'views/robots',
  'views/map',
  'views/robot',
  'views/plug',
  'views/navigate',
  'views/look',
  'views/client',
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
  PlugController,
  NavigateController,
  LookController,
  ApplicationView,
  NavigatingView,
  RobotsView,
  RobotView,
  PlugView,
  NavigateView,
  LookView,
  Robot,
  Place,
  Helpers
) {

  function doTheAppThing( router, name, context, client_context) {
    clientObj = App.Client.find(client_context);
    router.get('applicationController')
      .connectOutlet({
        outletName: 'client',
        name: 'client',
        context: clientObj
      });
    router.get('applicationController')
      .connectOutlet({
        outletName: 'content',
        name: name,
        context: context
      });
    router.get(name+'Controller').set('client',clientObj)
  };

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
          doTheAppThing( router, 'robots', App.Robot.find({format:'json'}), 'robots');
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
        look: Ember.Route.transitionTo('look'),

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
          /* And also update the client status information view with the new
           * application context (i.e.: where the user is now in the app) */
          doTheAppThing( router, 'robot', this.robot, 'robot:' + context.id);
          /* Set the RobotView's {{outlet}} to be a NavigateView with
           * a NagivateController which has a Robot model as context */
          router.get('robotController')
            .connectOutlet('navigate', this.robot);
          /* Set the NagivateView's {{outlet}} to be a MapView with a
             MapController using a Place model as context */
          router.get('navigateController')
            .connectOutlet('map', Ember.Object.create(
              {
              'enablePlaces': true,
              'enableRobot': true,
              'robot': this.robot,
              'places': App.Place.find({format:'json'})
              }
            ));
        }
      }),

      plug : Ember.Route.extend({
        route: '/robots/:id/plug',
        showAllRobots: Ember.Route.transitionTo('robots'),

        navigate: Ember.Route.transitionTo('navigate'),
        look: Ember.Route.transitionTo('look'),

        connectOutlets: function(router, context) {
          doTheAppThing( router, 'robot', App.Robot.find(context.id), 'robot:' + context.id);
          router.get('robotController')
            .connectOutlet('plug', App.Robot.find(context.id));
        }
      }),

      look : Ember.Route.extend({
        route: '/robots/:id/look',
        showAllRobots: Ember.Route.transitionTo('robots'),

        navigate: Ember.Route.transitionTo('navigate'),
        plug: Ember.Route.transitionTo('plug'),

        connectOutlets: function(router, context) {
          doTheAppThing( router, 'robot', App.Robot.find(context.id), 'robot:' + context.id);
          router.get('robotController')
            .connectOutlet('look', App.Robot.find(context.id));
        }
      }),



      navigating: Ember.Route.extend({
        route: '/navigating/:robot_id/:place_id',
        showAllRobots: Ember.Route.transitionTo('robots'),
        plug: Ember.Route.transitionTo('plug'),
        navigate: Ember.Route.transitionTo('navigate'),
        look: Ember.Route.transitionTo('look'),

        connectOutlets: function(router, context) {
          this.robot = App.Robot.find(context.robot_id);
          this.place = App.Place.find(context.place_id);
          doTheAppThing( router, 'robot', this.robot, 'robot:' + context.robot_id);
          router.get('robotController').
            connectOutlet('navigating', Ember.Object.create({
              robot: this.robot,
              place: this.place
            }));
          router.get('navigatingController').
            connectOutlet('map', Ember.Object.create({
              enablePlaces: false,
              enableRobot: true,
              robot: this.robot,
              place: this.place,
              places: App.Place.find({format:'json'})
            }));
          // Send the robot the actual navigateTo command via ROS
          this.robot.navigateTo(this.place);
        }
      })
    })
  });
});
