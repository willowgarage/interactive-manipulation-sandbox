var myApp;
define([
  'ember',
  'emberdata',
  'app',
  'ROS',
  'action',
],
function( Ember, DS, App, ROS, Action) {
  myApp = App;
  App.Robot = DS.Model.extend({
    name: DS.attr('string'),
    description: DS.attr('string'),
    tags: DS.attr('string'),
    image: DS.attr('string'),
    state: DS.attr('number'),     //  Coming from the mid-tier, currently unused
    service_url: DS.attr('string'),
    camera_url: DS.attr('string'),
    forearm_camera_url: DS.attr('string'),

    status_code: 0,            //  Calculated in the client
    status: function() {
      switch(this.get('status_code')) {
        case 0:
          return { value: "unreachable", class: "disabled" }
        case 1:
          return { value: "connected", class: "ready" }
        case 2:
          return { value: "disconnected", class: "error" }
        default:
          return { value: "N/A", class: "error" }
      }
    }.property('status_code'),
    isConnected: function() {
      return (this.get('status_code') == 1);
    }.property('status_code'),

    battery: -1,
    plugged_in_value: -1,
    // Whether the robot is ready to act. Enabled is false if the runstop
    // is pressed.
    enabled: false,
    pose: { 'x': -1 , 'y': -1 },

    plugged_in: function() {
      return (this.get('plugged_in_value') > 0);
    }.property('plugged_in_value'),

    log: function( msg) {
      console.log("["+ this.get('name') + "]: " + msg);
    },

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

    serviceUrlChanged: function() {
      if(this.get('service_url')) {
        this.ros = new ROS();

        //myDebugEvents( this.ros, this.get('name'), ['connection','close','error']);

        this.ros.connect(this.get('service_url'));

        var _this = this;
        this.ros.on('connection',function() {
          _this.set('status_code', 1);

          _this.topic_dashboard = new _this.ros.Topic({
            name: '/dashboard_agg',
            messageType: 'pr2_msgs/DashboardState'
          });
          _this.topic_dashboard.subscribe(function(message) {
            _this.set('battery', message.power_state.relative_capacity);
            _this.set('plugged_in_value', message.power_state.AC_present);
            if (message.power_board_state) {
              if (message.power_board_state.wireless_stop && message.power_board_state.run_stop) {
                _this.set('enabled', true);
              } else {
                _this.set('enabled', false);
              }
            } else {
              _this.set('enabled', false);
            }
          });
          _this.ros.on('close',function() {
            _this.set('status_code',2);
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

      //myDebugEvents( action, this.get('name') + " navigateTo action", ['result','status','feedback']);
  
      action.inputs.x        = place.get('pose_x');
      action.inputs.y        = place.get('pose_y');
      action.inputs.theta    = place.get('pose_angle');
      action.inputs.frame_id = '/map';
      action.execute();
      console.log('Calling NavigateTo action');
    },

    /* Unplugging has three steps: remove the plug from the wall, tuck your
     * arms, and point the head forward. We call them in sequence, checking
     * for a successful result after each step. */
    unplug: function() {
      var action = new Action({
        ros: this.ros,
        name: 'Unplug'
      })
      myDebugEvents( action, this.get('name') + " unplug action", ['result', 'feedback']);

      var _this = this;
      action.on("result", function(result) {
        console.log("unplug result: " + result.outcome);
        if (result.outcome == "succeeded") {
          // Unplug worked, now tuck arms
          _this._tuckArms();
        } else {
          // TODO: notify the user that unplugging failed
        }
      });
      action.execute();
      console.log("Calling Unplug action");
    },

    _tuckArms: function() {
      var action = new Action({
        ros: this.ros,
        name: 'TuckArms'
      })

      var _this = this;
      action.on("result", function(result) {
        console.log("tuckarms result: " + result.outcome);
        if (result.outcome == "succeeded") {
          // Tuckarms worked, now move head forward
          _this._pointHeadForward();
        } else {
          // TODO: Notify user that unplugging failed
        }
      });
      action.execute();
      console.log("Calling TuckArms action");
    },

    _pointHeadForward: function() {
      console.log("Point head forward is not implemented");
    },

    plugIn: function() {
      var action = new Action({
        ros: this.ros,
        name: 'PlugIn'
      });
      //myDebugEvents( action, this.get('name') + " plugIn action", ['result','status','feedback']);
      myDebugEvents( action, this.get('name') + " plugIn action", ['result', 'feedback']);
      action.on("result", function(result) {
        console.log("plugIn result: " + result.outcome);
      });
      action.execute();
      console.log("Calling PlugIn action");
    }
  });
});
function myDebugEvents( source, id, events) {
  for(var i=0;i<events.length;i++) {
    eval('var f = function(e){ console.log("["+id+".'+events[i]+'] received"); console.dir(arguments); };');
    source.on(events[i], f);
  }
};

