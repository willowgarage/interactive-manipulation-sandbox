define([
  'ember',
  'emberdata',
  'app',
  'ros'
],
function(
  Ember,
  DS,
  App,
  ROS
) {

  App.Robot = DS.Model.extend({
    name: DS.attr('string'),
    description: DS.attr('string'),
    tags: DS.attr('string'),
    image: DS.attr('string'),
    state: DS.attr('number'),
    service_url: DS.attr('string'),
    camera_url: DS.attr('string'),
    battery: -1,

    openRosConnection: Ember.observer(function(obj, keyName, value) {
      if (!obj.ROS && obj.get(keyName)) {
        obj.ROS = new ROS(obj.get('service_url'));
        var topic = new obj.ROS.Topic({
          name: '/dashboard_agg',
          messageType: 'pr2_msgs/DashboardState'
        });
        topic.subscribe(function(msg){
          console.log("Got a battery status update from robot = [" + obj.get('name') + "]");
          obj.set('battery',msg.power_state.relative_capacity);
        });
        console.log("Done suscribing to battery capacity for robot = [" + obj.get('name') + "]");
      }
    }, 'service_url'),

    navigateTo: function(place) {
      var action = new Action({
        ros: this.ROS,
        name: 'NavigateToPose'
      });
      action.inputs.x        = place.get('pose_x');
      action.inputs.y        = place.get('pose_y');
      action.inputs.theta    = place.get('pose_angle');
      action.inputs.frame_id = '/map';
      action.execute();
      console.log("Calling NavigateTo action");
    },

    unplug: function() {
      var action = new Action({
        ros: this.ROS,
        name: 'Unplug'
      });
      action.execute();
      console.log("Calling Unplug action");
    },

    plugIn: function() {
      var action = new Action({
        ros: this.ROS,
        name: 'PlugIn'
      });
      action.execute();
      console.log("Calling PlugIn action");
    }
  });

});

