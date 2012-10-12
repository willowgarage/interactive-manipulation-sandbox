define([
  'ember',
  'emberdata',
  'app',
  'ros',
  'action',
],
function( Ember, DS, App, ros, Action) {

  App.Robot = DS.Model.extend({
    name: DS.attr('string'),
    description: DS.attr('string'),
    tags: DS.attr('string'),
    image: DS.attr('string'),
    state: DS.attr('number'),
    service_url: DS.attr('string'),
    camera_url: DS.attr('string'),
    battery: -1,
    plugged_in_value: -1,
    plugged_in: function() {
      return (this.get('plugged_in_value') > 0);
    }.property('plugged_in_value'),

    serviceUrlChanged: function() {
      if(this.get('service_url')) {
        ros.connect(this.get('service_url'));
        var topic = new ros.Topic({
          name: '/dashboard_agg',
          messageType: 'pr2_msgs/DashboardState'
        });
        var _this = this;
        topic.subscribe(function(message) {
          _this.set('battery', message.power_state.relative_capacity);
          _this.set('plugged_in_value', message.power_state.AC_present);
        });
      }
    }.observes('service_url'),

    navigateTo: function(place) {
      var action = new Action({
        ros: ros,
        name: 'NavigateToPose'
      });
      action.inputs.x        = place.get('pose_x');
      action.inputs.y        = place.get('pose_y');
      action.inputs.theta    = place.get('pose_angle');
      action.inputs.frame_id = '/map';
      action.execute();
      console.log('Calling NavigateTo action');
    },

    unplug: function() {
      var action = new Action({
        ros: ros,
        name: 'Unplug'
      });
      action.execute();
      console.log("Calling Unplug action");
    },

    plugIn: function() {
      var action = new Action({
        ros: ros,
        name: 'PlugIn'
      });
      action.execute();
      console.log("Calling PlugIn action");
    }
  });
});

