/* Configuration variables */
var ROBOT_API = 'http://localhost:8000/world/api/robots?format=json-p'
var PLACE_API = 'http://localhost:8000/world/api/places?format=json-p'

/* Starting point for our Ember application */
var App = Ember.Application.create();

App.ApplicationView = Ember.View.extend({
	templateName: 'application'
});

App.ApplicationController = Ember.Controller.extend({
});

/* ---------------------------------------------------------------------- */
/* Robot controller */
App.AllRobotsController = Ember.ArrayController.extend();
App.AllRobotsView = Ember.View.extend({
	templateName: 'robots'
});

App.OneRobotController = Ember.ObjectController.extend();
App.OneRobotView = Ember.View.extend({
	templateName: 'a-robot'
});

/* ---------------------------------------------------------------------- */
/* Robot class */

App.Robot = Ember.Object.extend();
App.Robot.reopenClass({
	allRobots : [],
	find : function() {
		$.ajax({
			url: ROBOT_API,
			dataType: 'jsonp',
			context: this,
			success: function(response) {
				console.log(response);
				response.forEach(function(robot) {
					this.allRobots.addObject(App.Robot.create(robot))
				}, this)
			},
			error: function(response) {
				alert("Error retrieving robots!");
			}
		})
		return this.allRobots;
	},

	findOne: function(robot_id) {
		var robot = App.Robot.create({id: robot_id});

		$.ajax({
			url: ROBOT_API,
			dataType: 'jsonp',
			context: robot,
			success: function(response) {
				this.setProperties(response.findProperty('id', robot_id));
			}
		})

		return robot;
	},

});

/* ---------------------------------------------------------------------- */
/* Place controller */
App.AllPlacesController = Ember.ArrayController.extend();
App.AllPlacesView = Ember.View.extend({
	templateName: 'places'
});


/* ---------------------------------------------------------------------- */
/* Place class */

App.Place = Ember.Object.extend();
App.Place.reopenClass({
	allPlaces : [],
	find : function() {
		$.ajax({
			url: PLACE_API,
			dataType: 'jsonp',
			context: this,
			success: function(response) {
				console.log(response);
				response.forEach(function(place) {
					this.allPlaces.addObject(App.Place.create(place))
				}, this)
			}
		})
		return this.allPlaces;
	}
});

/* ---------------------------------------------------------------------- */
/* URL routing */
App.Router = Ember.Router.extend({
	root : Ember.Route.extend({
		index: Ember.Route.extend({
			route: '/',
			redirectsTo: 'robots'
		}),

		robots: Ember.Route.extend({
			route: '/robots',

			showRobot: Ember.Route.transitionTo('aRobot'),

			connectOutlets: function(router) {
				router.get('applicationController').
					connectOutlet('allRobots', App.Robot.find());
			}
		}),

		aRobot: Ember.Route.extend({
			route : '/robots/:robot_id',

			showAllRobots: Ember.Route.transitionTo('robots'),

			connectOutlets : function(router, context) {
				router.get('applicationController').
					connectOutlet('oneRobot', context);
			},
			serialize: function(router, context) {
				return { robot_id : context.get('id') }
			},
			deserialize: function(router, urlParams) {
				return App.Robot.findOne(urlParams.robot_id);
			}
		})
	})
});

/* Initialize our app. Must come at the end? */
App.initialize();

