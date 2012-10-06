/* Configuration variables */
var ROBOT_API = '/world/api/robots?format=json-p'
var PLACE_API = '/world/api/places?format=json-p'

/* Starting point for our Ember application */
var App = Ember.Application.create();


/* ---------------------------------------------------------------------- */
/* Overriding behavior in RESTAdapter to handle responses of the type
     [{ object },{ object }, ...]
   as returned by default in django-rest-framework 
   instead of currently required:
    { robots: [{ object },{ object }, ...]}

   NOTE: This can be greatly reduced in code size by generalizing the
   code that repeats between functions. I'm leaving it out for now
   to focus on the basics, but I should contribute this soon
*/

DS.DjangoRESTAdapter = DS.RESTAdapter.extend({
    findMany: function( store, type, ids) {
        ids = this.get('serializer').serializeIds(ids);
        var root = this.rootForType(type), plural = this.pluralize(root);
        this.ajax( this.buildURL(root), "GET", {
            data: { ids: ids },
            success: function( djson) {
                var json = {}; json[plural] = djson;
                this.sideload(store, type, json, plural);
                store.loadMany(type, json[plural]);
            }
        });
    },
    findAll: function( store, type) {
        var root = this.rootForType(type), plural = this.pluralize(root);
        this.ajax( this.buildURL(root), "GET", {
            success: function( djson) {
                var json = {}; json[plural] = djson;
                this.sideload(store, type, json, plural);
                store.loadMany(type, json[plural]);
            }
        });
    },
    findQuery: function( store, type, query, recordArray) {
        var root = this.rootForType(type), plural = this.pluralize(root);
        this.ajax( this.buildURL(root), "GET", {
            data: query,
            success: function( djson) {
                var json = {}; json[plural] = djson;
                this.sideload(store, type, json, plural);
                recordArray.load(json[plural]);
            }
        });
    }
});

App.store = DS.Store.create({ 
    revision: 6, 
    adapter: DS.DjangoRESTAdapter.create({ namespace: 'world/api' })
    //adapter: DS.RESTAdapter.create({ namespace: 'static/test' })
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

App.Robot = DS.Model.extend({
    //  ember-data mapping variables (?)
    name: DS.attr('string'),
    description: DS.attr('string'),
    tags: DS.attr('string'),
    image: DS.attr('string'),
    state: DS.attr('number'),
    service_url: DS.attr('string'),
    camera_url: DS.attr('string'),

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
/* Main application controller */

App.ApplicationController = Ember.Controller.extend({});
App.ApplicationView = Ember.View.extend({
	templateName: 'application'
});

/* ---------------------------------------------------------------------- */
/* Place controller */
App.AllPlacesController = Ember.ArrayController.extend();
App.AllPlacesView = Ember.View.extend({
	templateName: 'places'
});


/* ---------------------------------------------------------------------- */
/* Place class */

App.Place = DS.Model.extend({
    /* Field mappings */
    // TODO: See if this can be done automatically
    name: DS.attr('string'),
    description: DS.attr('string'),
    tags: DS.attr('string'),
    image: DS.attr('string'),
    pose_x: DS.attr('number'),
    pose_y: DS.attr('number'),
    pose_angle: DS.attr('number'),
    map_x: DS.attr('number'),
    map_y: DS.attr('number'),
    map_width: DS.attr('number'),
    map_height: DS.attr('number')
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
					connectOutlet('allRobots', App.Robot.find({format:'json'}));
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

