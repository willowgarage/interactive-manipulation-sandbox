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

App.OneRobotView = Ember.View.extend({
	templateName: 'a-robot'
});
App.OneRobotController = Ember.ObjectController.extend();

/* ---------------------------------------------------------------------- */
/* Robot class */

App.Robot = Ember.Object.extend();
App.Robot.reopenClass({
	allRobots : [],
	find : function() {
		$.ajax({
			url: 'http://localhost:8000/world/api/robots?format=json-p',
			dataType: 'jsonp',
			context: this,
			success: function(response) {
				console.log(response);
				response.forEach(function(robot) {
					this.allRobots.addObject(App.Robot.create(robot))
				}, this)
			}
		})
		return this.allRobots;
	},

	findOne: function(robotId) {
		var robot = App.Robot.create({id: robotId});

		$.ajax({
			url: 'http://localhost:8000/world/api/robots?format=json-p',
			dataType: 'jsonp',
			context: robot,
			success: function(response) {
				this.setProperties(response.findProperty('id', robotId));
			}
		})

		return robot;
	}
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
			url: 'http://localhost:8000/world/api/places?format=json-p',
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
		robots: Ember.Route.extend({
			route: '/',

			showRobot: Ember.Route.transitionTo('aRobot'),

			connectOutlets: function(router) {
				router.get('applicationController').
					connectOutlet('allRobots', App.Robot.find());
			}
		}),

		aRobot: Ember.Route.extend({
			route : '/:robotId',

			showAllRobots: Ember.Route.transitionTo('robots'),

			connectOutlets : function(router, context) {
				router.get('applicationController').
					connectOutlet('oneRobot', context);
			},
			serialize: function(router, context) {
				return { robotId : context.get('id') }
			},
			deserialize: function(router, urlParams) {
				return App.Robot.findOne(urlParams.robotId);
			}
		})
	})
});

/* Initialize our app. Must come at the end? */
App.initialize();

