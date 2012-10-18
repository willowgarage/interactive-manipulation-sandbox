var aROS;
define([
  'ember',
  'emberdata',
  'app',
  'ROS',
  'action',
],
function( Ember, DS, App, ROS, Action) {
  aROS = ROS;
  console.log("aROS ready");
  App.Robot = DS.Model.extend({
    name: DS.attr('string'),
    description: DS.attr('string'),
    tags: DS.attr('string'),
    image: DS.attr('string'),
    state: DS.attr('number'),     //  Coming from the mid-tier, currently unused
    service_url: DS.attr('string'),
    camera_url: DS.attr('string'),
    forearm_camera_url: DS.attr('string'),

    status: "Unknown",            //  Calculated in the client

    battery: -1,
    plugged_in_value: -1,
    pose: { 'x': -1 , 'y': -1 },
    plugged_in: function() {
      return (this.get('plugged_in_value') > 0);
    }.property('plugged_in_value'),

    map_coords: function() {
      var pose = this.get('pose');
      var map_x, map_y;
      if (pose.x == -1) {
        map_x = -1;
      } else {
        map_x = -3.2371041 * pose.x + -7.70845759 * pose.y + 564.53259318;
      }
      if (pose.y == -1) {
        map_y = -1;
      } else {
        map_y = -7.90508822 * pose.x + 3.38653133 * pose.y + 295.37609582;
      }
      return {'x': map_x, 'y': map_y};
    }.property('pose'),

    log: function( msg) {
      console.log("["+ this.get('name') + "]: " + msg);
    },

    serviceUrlChanged: function() {
      if(this.get('service_url')) {
        this.ros = new ROS();

        myDebugEvents( this.ros, this.get('name'), ['connection','close','error']);

        this.ros.connect(this.get('service_url'));

        var _this = this;
        this.ros.on('connection',function() {
          _this.set('status','Connected');

          _this.topic_dashboard = new _this.ros.Topic({
            name: '/dashboard_agg',
            messageType: 'pr2_msgs/DashboardState'
          });
          _this.topic_dashboard.subscribe(function(message) {
            _this.set('battery', message.power_state.relative_capacity);
            _this.set('plugged_in_value', message.power_state.AC_present);
          });
          _this.ros.on('close',function() {
            alert("WARNING: Connection to robot " + _this.get('name') + " lost");
            _this.set('status','Disconnected');
            _this.topic_dashboard.unsubscribe();
          });

          // Subscribe to pose messages
          _this.topic_pose = new _this.ros.Topic({
            name: '/robot_pose',
            messageType: 'geometry_msgs/PoseWithCovarianceStamped'
          });
          _this.topic_pose.subscribe(function(message) {
            _this.set('pose', {
              'x' : message.pose.pose.position.x,
              'y' : message.pose.pose.position.y
              });
          });
          _this.ros.on('close',function() {
            _this.topic_pose.unsubscribe();
          });
        });
      }
    }.observes('service_url'),

    navigateTo: function(place) {
      var action = new Action({
        ros: this.ros,
        name: 'NavigateToPose'
      });

      myDebugEvents( action, this.get('name') + " navigateTo action", ['result','status','feedback']);
  
      action.inputs.x        = place.get('pose_x');
      action.inputs.y        = place.get('pose_y');
      action.inputs.theta    = place.get('pose_angle');
      action.inputs.frame_id = '/map';
      action.execute();
      console.log('Calling NavigateTo action');
    },

    unplug: function() {
      var action = new Action({
        ros: this.ros,
        name: 'Unplug'
      });
      myDebugEvents( action, this.get('name') + " unplug action", ['result','status','feedback']);
      action.execute();
      console.log("Calling Unplug action");
    },

    plugIn: function() {
      var action = new Action({
        ros: this.ros,
        name: 'PlugIn'
      });
      myDebugEvents( action, this.get('name') + " plugIn action", ['result','status','feedback']);
      action.execute();
      console.log("Calling PlugIn action");
    }
  });
});
function myDebugEvents( source, id, events) {
  for(var i=0;i<events.length;i++) {
    source.on(events[i], function( e) {
      console.log("["+id+"."+e.type+"] received");
      console.dir(arguments);
    });
  }
};

