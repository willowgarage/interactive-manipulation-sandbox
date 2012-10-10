define([
  'ember',
  'app',
  'd3',
  'text!templates/robot.handlebars'
],
function(
  Ember,
  App,
  d3,
  robotHtml
) {

  App.RobotView = Ember.View.extend({
    template: Ember.Handlebars.compile(robotHtml),

    didInsertElement: function() {
      var robot_id = this.get('controller').get('content').get('id');
      var w = 480;
      var h = 374;
      var x = d3.scale.linear().domain([0, w]);
      var y = d3.scale.ordinal().domain([0, h]);

      // x, y, width, height, name, place-id
      var rooms = [
        [310, 338, 29, 34, "Tessa's office", "506dd79f52d6f70c3b8adfbd"],
        [280, 348, 28, 23, "Chad's office", "506dd9ca52d6f70c3b8adfbf"],
        [341, 339, 28, 33, "Brandon's office", "506ddb2252d6f70c3b8adfc1"],
        [259, 248, 50, 32, "The Cave", "506ddbed52d6f70c3b8adfc3"],
        [307, 149, 42, 97, "The Green Room", "506ddca152d6f70c3b8adfc5"],
        //[279, 50, 181, 78, "The cafeteria", "506dde9852d6f70c3b8adfc7"],
        [187, 235, 70, 75, "The pool room", "506ddee352d6f70c3b8adfc9"],
        [72, 190, 102, 73, "The Cathedral", "506ddf2452d6f70c3b8adfcb"]
      ];

      var svg = d3.select('#floorplan-div')
        .append('svg')
        .attr('width', w)
        .attr('height', h);

      svg.append('svg:image')
        .attr('xlink:href', '/static/willow-floorplan.png')
        .attr('width', w)
        .attr('height', h);

      svg.selectAll('.room')
        .data(rooms)
        .enter().append('svg:rect')
          .attr('class', 'room')
          .attr('x', function(d) { return d[0]; })
          .attr('y', function(d) { return d[1]; })
          .attr('width', function(d) { return d[2]; })
          .attr('height', function(d) { return d[3]; })
          .on('mouseover', function(d) {
            d3.select('#placename').text(d[4]);
          })
          .on('mouseout', function() {
            d3.select('#placename').text('');
          })
          .on('click', function(d) {
            App.get('router').send('navigateTo', {
              robot_id: robot_id,
              place_id: d[5]
            });
          });
    }
  });

});


