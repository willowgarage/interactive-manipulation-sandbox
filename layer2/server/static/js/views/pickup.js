define([
  'ember',
  'app',
 	'd3',
  'text!templates/pickup.handlebars'
], function( Ember, App, d3, pickupHtml) {

	App.PickupView = Ember.View.extend({
			template: Ember.Handlebars.compile(pickupHtml),

			// Callback when the content is finshed loading
			didInsertElement: function() {
				var canvas = document.getElementById("canvas");
				var ctx = canvas.getContext('2d');

				// Draw a rectangle on the canvas
				ctx.strokeStyle = "#00FF00";
				ctx.lineWidth = 2;
				ctx.rect(10, 10, 40, 30);
				ctx.stroke();

				var robot = this.get('controller').get('content');
        robot.addObserver('recognized_objects', this, 'drawObjects');
			},

			drawObjects: function(robot) {
				var objects = robot.get('recognized_objects');
				console.log("in drawObjects, objects:", objects);
				console.log("number of objects:", objects.length);

				var canvas = document.getElementById('canvas');
				var ctx = canvas.getContext('2d');

				ctx.clearRect(0, 0, canvas.width, canvas.height);

				for (obj_id in objects) {
					// Draw a rectangle on the canvas
					var obj = objects[obj_id];
					ctx.strokeStyle = "#00FF00";
					ctx.lineWidth = 2;
					var xmin = obj.xmin * canvas.width;
					var ymin = obj.ymin * canvas.height;
					var xmax = obj.xmax * canvas.width;
					var ymax = obj.ymax * canvas.height;
					var img_width = xmax - xmin;
					var img_height = ymax - ymin;
					console.log("Drawing", xmin, ymin, img_width, img_height);
					ctx.strokeRect(xmin, ymin, img_width, img_height);
				}

			}

    });

});
