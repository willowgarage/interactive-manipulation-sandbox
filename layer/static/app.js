/* Configuration variables */
var ROBOT_API = '/world/api/robots?format=json-p'
var PLACE_API = '/world/api/places?format=json-p'

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

	drawMap : function() {
		var w = 480,
		  h = 374,
		  x = d3.scale.linear().domain([0, w])
		  y = d3.scale.ordinal().domain([0, h]);

		// x, y, width, height, name, place-id
		var rooms = [
		  [310, 338, 29, 34, "Tessa's office", 1],
		  [280, 348, 28, 23, "Chad's office", 2],
		  [341, 339, 28, 33, "Brandon's office", 3],
		  [259, 248, 50, 32, "The Cave", 4],
		  [307, 149, 42, 97, "The Green Room", 5],
		//  [279, 50, 181, 78, "The cafeteria", 6],
		  [187, 235, 70, 75, "The pool room", 7],
		  [72, 190, 102, 73, "The Cathedral", 8]
		];
		/* TODO: get room data from an API
		var rooms = d3.json("<%= url_for(:controller => :room, :action => :list) %>");
		alert(rooms);
		*/
		  var svg = d3.select("#floorplan-div").append("svg")
			.attr("width", w)
			.attr("height", h);

		  svg.append("svg:image")
			.attr("xlink:href", "/static/willow-floorplan.png")
			.attr("width", w)
			.attr("height", h);

		  svg.selectAll(".room")
			.data(rooms)
			.enter().append("svg:rect")
			  .attr("class", "room")
			  .attr("x", function(d) { return d[0]; })
			  .attr("y", function(d) { return d[1]; })
			  .attr("width", function(d) { return d[2]; })
			  .attr("height", function(d) { return d[3]; })
			  .on("mouseover", function(d) {
				d3.select("#placename").text(d[4]);
				})
			  .on("mouseout", function() {
				d3.select("#placename").text("");
				})
			  .on("click", function(d) {
				var url = "/change/me" + "/" + d[5];
				window.location = url;
				});
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

