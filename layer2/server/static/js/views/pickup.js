define([
  'ember',
  'app',
  'd3',
  'text!templates/pickup.handlebars'
], function( Ember, App, d3, pickupHtml) {

  App.PickupView = Ember.View.extend({
      template: Ember.Handlebars.compile(pickupHtml),
      width: 400,
      height: 300,

      // Callback when the content is finshed loading
      didInsertElement: function() {
        // Create the SVG element in the canvas div
        var svg = d3.select("#canvas").append("svg")
          .attr("width", this.width)
          .attr("height", this.height)
          .attr("id","svgcanvas");

        // Set up the listener so that when segment_and_recognize finishes, we draw the objects
        var robot = this.get('controller').get('content');
        robot.addObserver('recognized_objects', this, 'drawObjects');

      },

      drawObjects: function(robot) {
        var robot = this.get('controller').get('content');
        var objects = robot.get('recognized_objects');

        var svg = d3.select("#svgcanvas");

        var data = new Array();
        for (obj_id in objects) {
          var obj = objects[obj_id];
          obj['id'] = obj_id;
          data.push(obj);
        }

        var _this = this;
        svg.selectAll(".recognized_object")
          .data(data)
          .enter().append("svg:rect")
          .attr("class", "recognized_object")
          .attr("x", function(d) { return d.xmin * _this.width; })
          .attr("y", function(d) { return d.ymin * _this.height; })
          .attr("width", function(d) { return (d.xmax - d.xmin) * _this.width; })
          .attr("height", function(d) { return (d.ymax - d.ymin) * _this.height; })
          .attr("style", "stroke: #00FF00; fill: none; stroke-width: 2;");

      }

    });

});
