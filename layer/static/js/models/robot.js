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
    progress_update: "",

    battery: -1,
    plugged_in_value: -1,
    // Whether the robot is ready to act. Enabled is false if the runstop
    // is pressed.
    enabled: false,
    pose: { 'x': -1 , 'y': -1 },
    // TL: hack because I can't get Handlebars expressions to work
    pose_display: "",
    // True iff the robot is in the process of plugging in
    is_plugging_in: false,

    plugged_in: function() {
      return (this.get('plugged_in_value') > 0);
    }.property('plugged_in_value'),

    log: function( msg) {
      console.log("["+ this.get('name') + "]: " + msg);
    },

    transformPoseToMap : function(pose) {
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
    },

    map_coords: function() {
      var pose = this.get('pose');
      return this.transformPoseToMap(pose);
    }.property('pose'),

    // TL: Define a derived property that will be used to display the
    // robot's current pose in the robot view. This is a hack! Ideally it
    // would be done using a Handlebars expression. But I wasn't able to
    // get that to work after several hours of trying.
    pose_display: function() {
      var pose = this.get('pose');
      if (pose.x == -1 && pose.y == -1) {
        return "unknown";
      } else {
        return pose.x.toFixed(2) + ', ' + pose.y.toFixed(2);
      }
    }.property('pose'),

    // Empty navigation plan
    navigation_plan: { poses: [] },

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

          // Subscribe to navigation plans
          _this.topic_navplan = new _this.ros.Topic({
            name: '/plan_throttled',
            messageType: 'nav_msgs/Path'
          });
          _this.topic_navplan.subscribe(function(message) {
            _this.set('navigation_plan', {
              'poses': message.poses
            });
          });
          _this.ros.on('close',function() {
            _this.topic_navplan.unsubscribe();
          });

        });
      }
    }.observes('service_url'),

    navigateTo: function(place) {
      // Sanity checks to make sure we are navigating to a reasonable place
      if (!place.get('pose_x') || !place.get('pose_y')) {
        this.set('progress_update', 'Invalid navigation coordinates');
        // Redirect to navigate view. We probably got here because the user
        // reloaded in the navigating view
        App.get('router').send("navigate", this);
        return;
      }

      // Make sure we aren't plugged in first
      if (this.get('plugged_in')) {
        this.set('progress_update', 'Please unplug before navigating');
        return;
      }

      this.set('progress_update', 'Tucking arms...');

      var action = new Action({
        ros: this.ros,
        name: 'TuckArms'
      })

      var _this = this;
      action.on("result", function(result) {
        console.log("Received result from tucking arms: ", result);
        _this.set('progress_update', 'Tucking arms ' + result.outcome);
        if (result.outcome == "succeeded") {
          // Tuckarms worked, now go
          _this._navigateTo2(place);
        } else {
          _this.set('progress_update', 'Arms not tucked, navigating anyway');
        }
      });
      console.log("Sending TuckArm action");
      action.execute();
    },

    _navigateTo2: function(place) {
      this.set('progress_update', 'Navigating to ' + place.get('name'));
      var action = new Action({
        ros: this.ros,
        name: 'NavigateToPose'
      });

      // Get notified when navigation finishes
      var _this = this;
      action.on("result", function(result) {
        console.log("navigation result: " + result.outcome);
        _this.set('progress_update', 'Navigation ' + result.outcome);
        // Return to navigation view
        App.get('router').send("navigate", _this);
      });
  
      action.inputs.x        = place.get('pose_x');
      action.inputs.y        = place.get('pose_y');
      action.inputs.theta    = place.get('pose_angle');
      action.inputs.frame_id = '/map';
      console.log("Sending navigation action:", action.inputs);
      action.execute();
    },

    /* Sending a Cancel goal (ExecuteAction is currently a SimpleActionState, 
     * so that should cancel any previous goals) */ 
    cancelAllGoals: function() {
      this.set('progress_update', 'Cancelling all goals');

      var action = new Action({
        ros: this.ros
      });
      action.cancelAll();
      console.log("Cancelling all goals");
    },

    /* Unplugging has three steps: remove the plug from the wall, tuck your
     * arms, and point the head forward. We call them in sequence, checking
     * for a successful result after each step. */
    unplug: function() {
      this.set('progress_update', 'Unplugging...');
      // Set the UI to say that unplugging is in progress
      this.set('is_plugging_in', true);

      var action = new Action({
        ros: this.ros,
        name: 'Unplug'
      })

      var _this = this;
      action.on("result", function(result) {
        console.log("unplug result: " + result.outcome);
        _this.set('progress_update', 'Unplugging ' + result.outcome);
        if (result.outcome == "succeeded") {
          // Unplug worked, now tuck arms
          _this._tuckArms();
        } else {
          // TODO: notify the user that unplugging failed
          // Point the head forwards
          _this._pointHeadForward();
        }
      });
      action.execute();
      console.log("Calling Unplug action");
    },

    _tuckArms: function() {
      this.set('progress_update', 'Tucking arms...');
      this.set('is_plugging_in', true);

      var action = new Action({
        ros: this.ros,
        name: 'TuckArms'
      })

      var _this = this;
      action.on("result", function(result) {
        console.log("tuckarms result: " + result.outcome);
        _this.set('progress_update', 'Tucking arms ' + result.outcome);
        if (result.outcome == "succeeded") {
          // Tuckarms worked, now move head forward
          _this._pointHeadForward();
        } else {
          // TODO: Notify user that unplugging failed
          _this.set('is_plugging_in', false);
        }
      });
      action.execute();
      console.log("Calling TuckArms action");
    },

    _pointHeadForward: function() {
      this.set('progress_update', 'Looking forward...');
      var action = new Action({
        ros: this.ros,
        name: 'PointHead'
      });

      // Get notified when head movement finishes
      var _this = this;
      action.on("result", function(result) {
        _this.set('progress_update', 'Look forward ' + result.outcome);
        // Regardless of outcome, tell the UI to say that plugging is done
        _this.set('is_plugging_in', false);
      });
  
      action.inputs.target_frame        = 'torso_lift_link';
      action.inputs.target_x            = 1.0;
      action.inputs.target_y            = 0;
      action.inputs.target_z            = 0.4;
      action.inputs.pointing_frame      = 'head_mount_kinect_rgb_optical_frame';
      action.inputs.pointing_x          = 0.0;
      action.inputs.pointing_y          = 0.0;
      action.inputs.pointing_z          = 1.0;
      action.execute();
      console.log('Calling PointHead action');
    },

    pointHead: function(x, y, z) {
      this.set('progress_update', 'Looking ...');
      var action = new Action({
        ros: this.ros,
        name: 'PointHead'
      });

      action.inputs.target_frame        = 'wide_stereo_link';
      action.inputs.target_x            = x;
      action.inputs.target_y            = y;
      action.inputs.target_z            = z;
      action.inputs.pointing_frame      = 'wide_stereo_link';
      action.inputs.pointing_x          = 1.0;
      action.inputs.pointing_y          = 0;
      action.inputs.pointing_z          = 0;
      action.execute();
      console.log('Calling PointHead action with ' + x + '/' + y + '/' + z);
    },

    plugIn: function() {
      this.set('progress_update', 'Plugging in...');
      // Set the UI to say that plugging is in progress
      this.set('is_plugging_in', true);

      var action = new Action({
        ros: this.ros,
        name: 'PlugIn'
      });
      var _this = this;
      //myDebugEvents( action, this.get('name') + " plugIn action", ['result','status','feedback']);
      myDebugEvents( action, this.get('name') + " plugIn action", ['result', 'feedback']);
      action.on("result", function(result) {
        console.log("plugIn result: " + result.outcome);
        _this.set('progress_update', 'Plugging in ' + result.outcome);
        _this.set('is_plugging_in', false);

        if (result.outcome == "succeeded") {
          // TODO: notify user that plugging in finished
          // Tell the UI to say that plugging is done
          _this.set('is_plugging_in', false);
        } else {
          // TODO: notify the user that plugging in failed
          // and point our head forwards
          _this._pointHeadForward();
        }

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

