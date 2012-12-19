define([
  'ember',
  'app',
  'jquery',
  'd3',
  'text!templates/pickup.handlebars'
], function(Ember, App, $, d3, pickupHtml) {

  App.PickupView = Ember.View.extend({
      template: Ember.Handlebars.compile(pickupHtml),
      width: 400,
      height: 300,

      // Callback when the content is finshed loading
      didInsertElement: function() {
        // Create the SVG element in the canvas div
        var svg = d3.select('#canvas').append('svg')
          .attr('width', this.width)
          .attr('height', this.height)
          .attr('id','svgcanvas');

        // Set up the listener so that when segment_and_recognize finishes, we draw the objects
        var robot = this.get('controller').get('content');
        robot.addObserver('recognized_objects', this, 'drawObjects');

        // Make sure nothing is currently selected
        this.get('controller').set('selected_object', null);
      },

      willDestroyElement: function() {
        // Remove the recognized_objects listener
        var robot = this.get('controller').get('content');
        robot.removeObserver('recognized_objects', this, 'drawObjects');
      },

      drawObjects: function(ev) {
        /* When the user clicks on an object, update "Selected object" and
         * store this object id in our controller */
        function objSelected(d) {
          console.log('in objSelected, obj:', d);

          // Remove old selection
          d3.select('.selected').classed('selected', false);
          // Add new selection
          d3.select(d3.event.target).classed('selected', true);

          // Store this object id in our controller
          _this.get('controller').set('selected_object', d.id);
        }

        var pickupController = this.get('controller');
        var robot = pickupController.get('content');
        var objects = robot.get('recognized_objects');

        var svg = d3.select('#svgcanvas');

        var data = [];
        for (var obj_id in objects) {
          var obj = objects[obj_id];
          obj['id'] = obj_id;
          data.push(obj);
        }

        var _this = this;
        svg.selectAll('.recognized_object')
          .data(data)
          .enter().append('svg:rect')
          .attr('class', 'recognized_object')
          .attr('x', function(d) { return d.xmin * _this.width; })
          .attr('y', function(d) { return d.ymin * _this.height; })
          .attr('width', function(d) { return (d.xmax - d.xmin) * _this.width; })
          .attr('height', function(d) { return (d.ymax - d.ymin) * _this.height; })
          .on('click', objSelected);
      }

    });
});
