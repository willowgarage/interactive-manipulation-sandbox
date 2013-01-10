define([
  'ember',
  'app',
  'jquery',
  'd3',
  'text!templates/map.handlebars'
], function(Ember, App, $, d3, mapHtml) {

  App.MapView = Ember.View.extend({
    template: Ember.Handlebars.compile(mapHtml),

    enablePlaces: true,
    enableRobot: true,

    didInsertElement: function() {
      var w = 480,
        h = 374,
        x = d3.scale.linear().domain([0, w]),
        y = d3.scale.ordinal().domain([0, h]);

      var svg = d3.select('#floorplan-div').append('svg')
        .attr('width', w)
        .attr('height', h)
        .attr('id', 'mapsvg');

      svg.append('svg:image')
        .attr('xlink:href', '/static/img/willow-floorplan.png')
        .attr('width', w)
        .attr('height', h);

      var content = this.get('controller').get('content');
      this.enablePlaces = content.get('enablePlaces');
      this.enableRobot = content.get('enableRobot');
      // Whether to draw the current destination on the map
      this.enableDestination = content.get('enableDestination');

      // Observer so that when the places in the database change, we update
      // the map
      if (this.enablePlaces) {
        var places = content.places;
        places.addArrayObserver(this);
      }
      else {
        // If we are not enabling Places, then hide the nav panel
        $('#navigation_panel').hide();
      }

      // Add an observer so that whenever the robot's position changes, we
      // update the map
      if (this.enableRobot) {
        var robot = content.robot;
        robot.addObserver('map_coords', this, 'drawRobot');
        robot.addObserver('navigation_plan', this, 'drawNavPlan');
        // Draw the robot now, because it'll be a couple seconds before the
        // location updates
        this.drawRobot(content.robot);
      }

      if (this.enableDestination) {
        // Draw the destination on the map
        this.drawDestination(content.place);
      }
    },

    willDestroyElement: function() {
      var content = this.get('controller').get('content');

      // Remove listeners on places and this robot
      if (this.enablePlaces) {
        var places = content.places;
        places.removeArrayObserver(this);
      }

      if (this.enableRobot) {
        var robot = content.robot;
        robot.removeObserver('map_coords', this, 'drawRobot');
        robot.removeObserver('navigation_plan', this, 'drawNavPlan');
      }

      // Remove the map so it gets redrawn next time
      d3.select('#mapsvg').remove();
    },

    arrayWillChange: function() {},
    arrayDidChange: function() {
      this.drawRooms();
    },

    drawRooms: function() {
      var content = this.get('controller').get('content');
      var robot = content.robot;
      var places = content.places.toArray();

      // Draw each room and outlet on the map
      this.drawPlaces(places);

      // Plot the robot on the map last, so that it comes out on top
      // TODO: signal that the robot's location has changed so that it will
      // be drawn on top of the rooms

      // Figure out which room the robot is closest to
      var nearest = -1;
      var closest_place = null;
      for (var i = 0; i < places.length; i++) {
        var place = places[i];
        if (place.get('map_x') === null || place.get('map_y') === null) {
          continue;
        }
        var robot_coords = robot.get('map_coords');

        // First check if the robot is inside
        var map_x = place.get('map_x');
        var map_y = place.get('map_y');
        var height = place.get('map_height');
        var width = place.get('map_width');
        var rx = robot_coords.x;
        var ry = robot_coords.y;
        if ((map_x - 1 <= rx) &&
          (rx <= (map_x + width + 1)) &&
          (map_y - 1 <= ry) &&
          (ry <= (map_y + height + 1))) {
            closest_place = place;
            break;
        }

        // Otherwise, find its distance to the nearest corner of the room
        var distance = this.getDistance(robot_coords, place);
        if ((nearest === -1) || distance < nearest) {
          nearest = distance;
          closest_place = place;
        }
      }
      if (closest_place !== null) {
        d3.select('#closest').text(closest_place.get('name'));
      }
    },

    getDistance : function(robot, place) {
      var rx = robot.x;
      var ry = robot.y;
      var px = place.get('map_x');
      var py = place.get('map_y');
      var h = place.get('map_height');
      var w = place.get('map_width');
      return Math.sqrt(Math.min(
        Math.pow(rx - px, 2) + Math.pow(ry - py, 2),
        Math.pow(rx - (px+h), 2) + Math.pow(ry - py, 2),
        Math.pow(rx - (px), 2) + Math.pow(ry - py+h, 2),
        Math.pow(rx - (px+h), 2) + Math.pow(ry - py+h, 2)));
    },

    drawRobot: function(robot) {
      var map = d3.select('#mapsvg');
      var _this = this;

      if ((robot.get('map_coords').x === -1) || (robot.get('map_coords').y === -1)) {
        return;
      }

      // Remove old robot
      map.selectAll('.robot').remove();

      /* Draw our robot on the map */
      map.selectAll('.robot')
        .data([robot])
        .enter().append('svg:circle')
          .attr('class', 'robot')
          .attr('cx', function(d) {
              return d.get('map_coords').x;
            })
          .attr('cy', function(d) {
              return d.get('map_coords').y;
            })
          .attr('r', 5);
    },

    drawDestination : function(place) {
      this.drawPlaces([place]);
    },

    drawPlaces: function(places) {
      if (!places) {
        return;
      }

      var rooms = places.filter( function(p) { return (!p.get('isOutlet') && !p.get('isTable')); });
      var outlets = places.filter( function(p) { return p.get('isOutlet'); });
      var tables = places.filter( function(p) { return p.get('isTable'); });
      var map = d3.select('#mapsvg');

      /* When the user clicks on a room, update "Selected location" and
       * store this place name in our controller */
      function nodeSelected(d) {
        // Remove old selection
        d3.select('.selected').classed('selected', false);
        // Add new selection
        d3.select(d3.event.target).classed('selected', true);

        // Populate the "Selected location" text field
        d3.select('#placename').text(d.get('name'));
        // Set the selected place in the controller
        _this.get('controller').set('selectedPlace', d);
      }

      /* Draw Rooms */
      var _this = this;
      map.selectAll('.room')
        .data(rooms.toArray())
        .enter().append('svg:rect')
          .attr('class', 'room')
          .attr('x', function(d) { return d.get('map_x'); })
          .attr('y', function(d) { return d.get('map_y'); })
          .attr('width', function(d) { return d.get('map_width'); })
          .attr('height', function(d) { return d.get('map_height'); })
          .on('click', nodeSelected);

      /* Draw Outlets */
      map.selectAll('.outlet')
        .data(outlets.toArray())
        .enter().append('svg:image')
          .attr('class', 'outlet')
          .attr('xlink:href', '/static/img/outlet.jpg')
          .attr('x', function(d) { return d.get('map_x'); })
          .attr('y', function(d) { return d.get('map_y'); })
          .attr('width', 20)
          .attr('height', 20)
          .on('click', nodeSelected);

      /* Draw Tables */
      map.selectAll('.table')
        .data(tables.toArray())
        .enter().append('svg:image')
          .attr('class', 'table')
          .attr('xlink:href', '/static/img/table.png')
          .attr('x', function(d) { return d.get('map_x'); })
          .attr('y', function(d) { return d.get('map_y'); })
          .attr('width', 20)
          .attr('height', 20)
          .on('click', nodeSelected);
    },

    drawNavPlan : function() {
      var robot = this.get('controller').get('content').robot;

      var plan = robot.get('navigation_plan');

      var poses = plan.poses;

      var map = d3.select('#mapsvg');
      // Remove the old plan
      map.selectAll('.navpath').remove();

      if (poses.length === 0) {
        return;
      }

      // Construct an array of points
      var array1 = poses.slice(0, poses.length - 1);
      var array2 = poses.slice(1, poses.length);
      var poseSegments = d3.zip(array1, array2);

      // Draw the new plan
      map.selectAll('.navpath')
        .data(poseSegments)
        .enter().append('svg:line')
        .attr('x1', function(d) { return robot.transformPoseToMap(d[0].pose.position).x; })
        .attr('y1', function(d) { return robot.transformPoseToMap(d[0].pose.position).y; })
        .attr('x2', function(d) { return robot.transformPoseToMap(d[1].pose.position).x; })
        .attr('y2', function(d) { return robot.transformPoseToMap(d[1].pose.position).y; })
        .attr('stroke', 'green')
        .attr('stroke-width', '2')
        .attr('fill', 'none')
        // Tagging them with a class name lets us remove them later
        .attr('class', 'navpath');

    }
  });

});
