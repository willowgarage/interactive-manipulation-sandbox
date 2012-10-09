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
    find: function( store, type, id) {
        var root = this.rootForType(type);
        this.ajax( this.buildURL(root, id), "GET", {
            success: function( djson) {
                var json = {}; json[root] = djson;
                this.sideload(store, type, json, root);
                store.load(type, json[root]);
            }
        });
    },
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
});

/* ---------------------------------------------------------------------- */
/* Place model */

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
/* Robot model */

App.Robot = DS.Model.extend({
    //  ember-data mapping variables (?)
    name: DS.attr('string'),
    description: DS.attr('string'),
    tags: DS.attr('string'),
    image: DS.attr('string'),
    state: DS.attr('number'),
    service_url: DS.attr('string'),
    camera_url: DS.attr('string'),

    battery: -1,

    openRosConnection: Ember.observer(function( obj, keyName, value) {
        if(!obj.ROS && obj.get(keyName)) {
            obj.ROS = new ROS(obj.get('service_url'));
            new obj.ROS.Topic({
                name: '/dashboard_agg',
                messageType: 'pr2_msgs/DashboardState'
            }).subscribe(function(msg){
                console.log("Got a battery status update from robot = [" + obj.get('name') + "]");
                obj.set('battery',msg.power_state.relative_capacity);
            });
            console.log("Done suscribing to battery capacity for robot = [" + obj.get('name') + "]");
        }
    }, 'service_url'),

    navigateTo: function( aPlace) {
        var action = new Action({
            ros: this.ROS,
            name: 'NavigateToPose'
        });
        action.inputs.x = aPlace.get('pose_x');
        action.inputs.y = aPlace.get('pose_y');
        action.inputs.theta = aPlace.get('pose_angle');
        action.inputs.frame_id = '/map';
        action.execute();
        console.log("Calling NavigateTo action");
    }
});

/* ---------------------------------------------------------------------- */
/* Robot views and controllers */
App.AllRobotsController = Ember.ArrayController.extend();
App.AllRobotsView = Ember.View.extend({
	templateName: 'robots'
});

App.OneRobotController = Ember.ObjectController.extend();
App.OneRobotView = Ember.View.extend({
	templateName: 'a-robot'
});

/* ---------------------------------------------------------------------- */
/* Map controller (child of OneRobot) */
App.MapController = Ember.ArrayController.extend({});
App.MapView = Ember.View.extend({
	templateName: 'map',
	didInsertElement: function() {
		var w = 480,
		  h = 374,
		  x = d3.scale.linear().domain([0, w])
		  y = d3.scale.ordinal().domain([0, h]);

		  var svg = d3.select("#floorplan-div").append("svg")
			.attr("width", w)
			.attr("height", h)
            .attr("id","damap");

		  svg.append("svg:image")
			.attr("xlink:href", "/static/willow-floorplan.png")
			.attr("width", w)
			.attr("height", h);

        this.get('controller').get('content').addArrayObserver( this);
    },
    arrayWillChange: function() {},
    arrayDidChange: function() {
        this.drawRooms();
    },
    drawRooms: function() {
        var rooms = this.get('controller').get('content');
        if(!rooms) {
            return;
        }
		var svg = d3.select("#damap").selectAll(".room")
			.data(rooms.toArray())
			.enter().append("svg:rect")
			  .attr("class", "room")
			  .attr("x", function(d) { return d.get('map_x'); })
			  .attr("y", function(d) { return d.get('map_y'); })
			  .attr("width", function(d) { return d.get('map_width'); })
			  .attr("height", function(d) { return d.get('map_height'); })
			  .on("mouseover", function(d) {
				d3.select("#placename").text(d.get('name'));
				})
			  .on("mouseout", function() {
				d3.select("#placename").text("");
				})
			  .on("click", function(d) {
                App.get('router').send("navigateTo",{
                        robot_id: App.router.get('oneRobotController').get('content').get('id'),
                        place_id: d.get('id')
                    });
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
/* Navigating controller and view */
App.NavigatingController = Ember.ObjectController.extend({
});
App.NavigatingView = Ember.View.extend({
	templateName: 'navigating'
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
			route: '/robots/:id',

			showAllRobots: Ember.Route.transitionTo('robots'),

			navigateTo: Ember.Route.transitionTo('navigating'),

			connectOutlets: function(router, context) {
				router.get('applicationController').
					connectOutlet('oneRobot', App.Robot.find(context.id));
				router.get('oneRobotController').
					connectOutlet('map', App.Place.find({format:'json'}));
			}
		}),

        navigating: Ember.Route.extend({
            route: '/navigating/:robot_id/:place_id',

			showAllRobots: Ember.Route.transitionTo('robots'),

            connectOutlets: function(router, context) {
                r = App.Robot.find(context.robot_id);
                p = App.Place.find(context.place_id);
                router.get('applicationController').
                    connectOutlet('navigating', Ember.Object.create({
                        robot: r, place: p
                    }));
                //  Not sure this is the right place for this to go
                //  but I can't find any place better.
                //  Make the robot actually navigate to the selected place
                r.navigateTo(p);
            }
        })
	})
});

/* Initialize our app. Must come at the end? */
App.initialize();

