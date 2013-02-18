var MyApp;
define([
  'ember',
  'emberdata',
  'app',
  'ROS',
  'action',
],
function( Ember, DS, App, ROS, Action) {
  MyApp = App;
  App.Robot = DS.Model.extend({
    name: DS.attr('string'),
    description: DS.attr('string'),
    tags: DS.attr('string'),
    image: DS.attr('string'),
    state: DS.attr('number'),     //  Coming from the mid-tier, currently unused
    service_url: DS.attr('string'),
    camera_base_url: DS.attr('string'),
    cameras: DS.attr('string'),

    // Attributes for keeping track of which camera the user wants to look through
    selected_camera: null,
    selectedCameraIsHead: function() {
      if (this.get('selected_camera') && this.get('selected_camera').name === 'head') {
        return true;
      }
      else {
        return false;
      }
    }.property('selected_camera'),

    //  Convenience pseudo-attribute function to get to the head camera
    camera_url: function() {
        return this.getCameraUrl('head');
    }.property('cameras'),

    //  Convenience pseudo-attribute function to get to the forearm
    forearm_camera_url: function() {
        return this.getCameraUrl('right arm');
    }.property('cameras'),

    //  Convenience pseudo-attribute function to get to the kinect camera for pickup
    pickup_camera_url: function() {
        return this.getCameraUrl('kinect');
    }.property('cameras'),

    // Subset of this.cameras which share the "look" feature.
    look_cameras: function() {
        var cameras = this.get('cameras'),
            LOOK_FEATURE = 'look',
            look_cameras = [];
        cameras && cameras.forEach(function(camera){
            camera.features.split(' ').forEach(function(feature){
                if (LOOK_FEATURE === feature){
                    look_cameras.push(camera)
                    return;
                }
            });
        });
        return look_cameras;
    }.property('cameras'),

    //  This method parses the camera URLs every time they change
    //  and makes them available to be queried by other helper methods
    camerasChanged: function() {
        var cameras = this.get('cameras');
        var camera_base_url = this.get('camera_base_url');
        cameras && cameras.forEach(function(camera){
            cameras[camera.name] = camera;
            camera.url = camera_base_url + camera.url;
        });
    }.observes('cameras'),

    //  Helper method to get the URL for a given camera name
    getCameraUrl: function(name) {
        return (this.get('cameras') && this.get('cameras')[name] &&
          this.get('cameras')[name].url);
    },
    // Helper to get the camera object by name
    getCameraByName: function(name) {
        return (this.get('cameras') && this.get('cameras')[name])
    },

    status_code: 0,            //  Calculated in the client
    status: function() {
      switch(this.get('status_code')) {
        case 0:
          return { value: 'unreachable', class: 'disabled' };
        case 1:
          return { value: 'connected', class: 'ready' };
        case 2:
          return { value: 'disconnected', class: 'error' };
        default:
          return { value: 'N/A', class: 'error' };
      }
    }.property('status_code'),
    isConnected: function() {
      return (this.get('status_code') === 1);
    }.property('status_code'),
    progress_update: '',

    // Whether the robot is ready to act. Enabled is false if the runstop
    // is pressed.
    pose: { 'x': -1 , 'y': -1 },
    // True iff the robot is in the process of plugging in
    is_plugging_in: false,

    plugged_in: function() {
      return (this.get('plugged_in_value') > 0);
    }.property('plugged_in_value'),

    log: function( msg) {
      console.log('['+ this.get('name') + ']: ' + msg);
    },

    transformPoseToMap : function(pose) {
      var map_x, map_y;
      if (pose.x === -1) {
        map_x = -1;
      }
      else {
        map_x = -3.2371041 * pose.x + -7.70845759 * pose.y + 564.53259318;
      }
      if (pose.y === -1) {
        map_y = -1;
      }
      else {
        map_y = -7.90508822 * pose.x + 3.38653133 * pose.y + 295.37609582;
      }
      return {'x' : map_x, 'y' : map_y};
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
      if (pose.x === -1 && pose.y === -1) {
        return 'unknown';
      }
      else {
        return pose.x.toFixed(2) + ', ' + pose.y.toFixed(2);
      }
    }.property('pose'),

    // Empty navigation plan
    navigation_plan: { poses: [] },

    // Clear statuses
    battery: null,
    plugged_in_value: null,
    runstop_activated: null,
    motors_not_halted: null,

    serviceUrlChanged: function() {
      if (this.get('service_url')) {
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
                _this.set('runstop_activated', false);
              } else {
                _this.set('runstop_activated', true);
              }
            } else {
              _this.set('runstop_activated', null);
            }

            if (message.motors_halted_valid) {
              _this.set('motors_not_halted', !(message.motors_halted.data));
            } else {
              _this.set('motors_not_halted', null);
            }
          });
          _this.ros.on('close',function() {
            _this.set('status_code',2);
            _this.topic_dashboard.unsubscribe();
          });

          // Subscribe to pose messages
          _this.topic_pose = new _this.ros.Topic({
            name: '/robot_pose',
            messageType: 'geometry_msgs/Pose'
          });
          _this.topic_pose.subscribe(function(message) {
            _this.set('pose', {
              'x' : message.position.x,
              'y' : message.position.y
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

    // ----------------------------------------------------------------------
    // Reset the motors

    reset_motors: function(ev) {
      // ros.Service provides an interface to calling ROS services.
      // Creates a rospy_tutorials/AddTwoInts service client named /add_two_ints.
      var reset_motors = new this.ros.Service({
        name        : '/pr2_etherCAT/reset_motors',
        serviceType : 'std_srvs/Empty'
      });

      // ros.ServiceRequest contains the data to send in the service call.
      var request = new this.ros.ServiceRequest();

      console.log('Sending reset_motors service call');
      reset_motors.callService(request, function(result) {
        // Callback when it finishes
        console.log('Result for reset_motors call:', result);
      });
    },


    // ----------------------------------------------------------------------
    // Navigation

    navigateTo: function(place) {
      // Sanity checks to make sure we are navigating to a reasonable place
      if (!place.get('pose_x') || !place.get('pose_y')) {
        this.set('progress_update', 'Invalid navigation coordinates');
        // Redirect to navigate view. We probably got here because the user
        // reloaded in the navigating view
        App.get('router').send('navigate', this);
        return;
      }

      // Make sure we aren't plugged in first
      if (this.get('plugged_in')) {
        this.set('progress_update', 'Please unplug before navigating');
        return;
      }

      // First tuck arms before navigating
      this.set('progress_update', 'Tucking arms...');

      var action = new Action({
        ros: this.ros,
        name: 'TuckArms'
      });
      action.inputs.tuck_left = true;
      action.inputs.tuck_right = true;

      var _this = this;
      action.on('result', function(result) {
        console.log('Received result from tucking arms: ', result);
        _this.set('progress_update', 'Tucking arms ' + result.outcome);
        if (result.outcome === 'succeeded') {
          // Tuckarms worked, now go
          _this._pointHeadForward(function() {
            _this._navigateTo2(place);
          });
        } else {
          _this.set('progress_update', 'Arms not tucked, navigating anyway');
        }
      });

      console.log('Sending TuckArm action');
      action.inputs.tuck_left = true;
      action.inputs.tuck_right = true;
      console.log('Sending TuckArm action');
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
      action.on('result', function(result) {
        console.log('navigation result: ' + result.outcome);
        _this.set('progress_update', 'Navigation ' + result.outcome);
        // Return to navigation view
        App.get('router').send('navigate', _this);
      });

      // Actually send the navigation command
      action.inputs.x        = place.get('pose_x');
      action.inputs.y        = place.get('pose_y');
      action.inputs.theta    = place.get('pose_angle');
      action.inputs.collision_aware    = true;
      action.inputs.frame_id = '/map';
      console.log('Sending navigation action:', action.inputs);
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
      console.log('Cancelling all goals');
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
      });

      var _this = this;

      var onError = function() {
        _this.set('progress_update', 'Unplugging failed');
        _this.set('is_plugging_in', false);
      };

      action.on('result', function(result) {
        console.log('unplug result: ' + result.outcome);
        _this.set('progress_update', 'Unplugging ' + result.outcome);
        if (result.outcome === 'succeeded') {
          // Unplug worked, now tuck arms
          _this._tuckArms(function() {
            _this._pointHeadForward(function() {
              _this.set('progress_update', 'Unplugging successful');
              _this.set('is_plugging_in', false);
            });
          }, onError);

        } else {
          _this.set('is_plugging_in', false);
        }
      });
      action.execute();
      console.log('Calling Unplug action');
    },

    _tuckArms: function(nextStep, onError) {
      this.set('progress_update', 'Tucking arms...');
      this.set('is_plugging_in', true);

      var action = new Action({
        ros: this.ros,
        name: 'TuckArms'
      });

      action.inputs.tuck_left = true;
      action.inputs.tuck_right = true;

      var _this = this;
      action.on('result', function(result) {
        console.log('tuckarms result: ' + result.outcome);
        _this.set('progress_update', 'Tucking arms ' + result.outcome);
        if (result.outcome === 'succeeded') {
          // Tuckarms worked, now move head forward
          if (nextStep) {
            nextStep();
          }
        } else {
          // TODO: Notify user that unplugging failed
          if (onError) {
            onError();
          }
        }
      });
      action.execute();
      console.log('Calling TuckArms action');
    },

    _pointHeadForward: function(nextStep, onError) {
      console.log('in _pointHeadForward, nextStep: ', nextStep);
      this.set('progress_update', 'Looking forward...');
      var action = new Action({
        ros: this.ros,
        name: 'PointHead'
      });

      // Get notified when head movement finishes
      var _this = this;
      action.on('result', function(result) {
        _this.set('progress_update', 'Look forward ' + result.outcome);
        // Regardless of outcome, tell the UI to say that plugging is done
        if (result.outcome === 'succeeded') {
          if (nextStep) {
            nextStep();
          }
        } else {
          if (onError) {
            onError();
          }
        }
      });

      action.inputs.target_frame   = 'torso_lift_link';
      action.inputs.target_x       = 1.0;
      action.inputs.target_y       = 0;
      action.inputs.target_z       = 0.4;
      action.inputs.pointing_frame = 'head_mount_kinect_rgb_optical_frame';
      action.inputs.pointing_x     = 0.0;
      action.inputs.pointing_y     = 0.0;
      action.inputs.pointing_z     = 1.0;
      action.execute();
      console.log('Calling PointHead action', action.inputs);
    },

    // Point the head at a location in the camera view
    pointHeadClick: function(fracX, fracY) {
      this.set('progress_update', 'Looking where you clicked ...');
      var action = new Action({
        ros: this.ros,
        name: 'PointHeadInImage'
      });

      action.inputs.target_x            = fracX;
      action.inputs.target_y            = fracY;
      action.inputs.camera_info_topic   = '/wide_stereo/left/camera_info';

      var _this = this;
      action.on('result', function(result) {
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', '');
        } else {
          _this.set('progress_update', 'Pointing the head failed');
        }
      });

      action.execute();
      console.log('Calling PointHeadInImage', action.inputs);

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
      //myDebugEvents( action, this.get('name') + ' plugIn action', ['result','status','feedback']);
      myDebugEvents( action, this.get('name') + ' plugIn action', ['result', 'feedback']);
      action.on('result', function(result) {
        console.log('plugIn result: ' + result.outcome);
        _this.set('progress_update', 'Plugging in ' + result.outcome);
        _this.set('is_plugging_in', false);

        if (result.outcome === 'succeeded') {
          // TODO: notify user that plugging in finished
          // Tell the UI to say that plugging is done
          _this.set('is_plugging_in', false);
        }
        else {
          // TODO: notify the user that plugging in failed
          // and point our head forwards
          _this._pointHeadForward(function() {
            _this.set('is_plugging_in', false);
          });

        }

      });
      action.execute();
      console.log('Calling PlugIn action');
    },

    // ----------------------------------------------------------------------
    // Move the base using open-loop control

    moveForward: function() {
      this.set('progress_update', 'Moving forward');
      var action = new Action({
        ros: this.ros,
        name: 'NavigateToPose'
      });

      action.inputs.x                  = 0.2;
      action.inputs.y                  = 0.0;
      action.inputs.theta              = 0.0;
      action.inputs.collision_aware    = false;
      action.inputs.frame_id           = '/base_footprint';

      var _this = this;
      action.on('result', function(result) {
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', '');
        }
        else {
          _this.set('progress_update', 'Move action failed');
        }
      });

      action.execute();
      console.log('Calling NavigateToPose action with parameters: ', action.inputs);
    },
    moveBack: function() {
      this.set('progress_update', 'Moving backward');
      var action = new Action({
        ros: this.ros,
        name: 'NavigateToPose'
      });

      action.inputs.x                  = -0.2;
      action.inputs.y                  = 0.0;
      action.inputs.theta              = 0.0;
      action.inputs.collision_aware    = false;
      action.inputs.frame_id           = '/base_footprint';

      var _this = this;
      action.on('result', function(result) {
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', '');
        }
        else {
          _this.set('progress_update', 'Move action failed');
        }
      });

      action.execute();
      console.log('Calling NavigateToPose action with parameters: ', action.inputs);
    },
    turnLeft: function() {
      this.set('progress_update', 'Turning left');
      var action = new Action({
        ros: this.ros,
        name: 'NavigateToPose'
      });

      action.inputs.x                  = 0.0;
      action.inputs.y                  = 0.0;
      action.inputs.theta              = 0.20;
      action.inputs.collision_aware    = false;
      action.inputs.frame_id           = '/base_footprint';

      var _this = this;
      action.on('result', function(result) {
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', '');
        }
        else {
          _this.set('progress_update', 'Move action failed');
        }
      });

      action.execute();
      console.log('Calling NavigateToPose action with parameters: ', action.inputs);
    },
    turnRight: function() {
      this.set('progress_update', 'Turning right');
      var action = new Action({
        ros: this.ros,
        name: 'NavigateToPose'
      });

      action.inputs.x                  = 0.0;
      action.inputs.y                  = 0.0;
      action.inputs.theta              = -0.20;
      action.inputs.collision_aware    = false;
      action.inputs.frame_id           = '/base_footprint';

      var _this = this;
      action.on('result', function(result) {
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', '');
        }
        else {
          _this.set('progress_update', 'Move action failed');
        }
      });

      action.execute();
      console.log('Calling NavigateToPose action with parameters: ', action.inputs);
    },

    // ----------------------------------------------------------------------
    // Manipulating objects in the world

    // These are the objects that segment-and-recognize has detected in front of the robot
    recognized_objects: {},

    segmentAndRecognize: function(pickupController) {
      this.set('progress_update', 'Resetting collision objects');
      var action = new Action({
        ros: this.ros,
        name: 'ResetCollisionObjects'
      });

      // Set parameters
      action.inputs.map = true;
      action.inputs.unattached_objects = true;
      action.inputs.attached_objects = true;
      action.inputs.arm = 'both';

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from ResetCollisionObjects:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this._segmentAndRecognize2(pickupController);
        }
        else {
          _this.set('progress_update', 'Failed to reset collision objects');
        }
      });

      console.log('Calling SegmentAndRecognize');
      action.execute();
    },

    _segmentAndRecognize2: function(pickupController) {
      this.set('progress_update', 'Identifying objects in view');
      var action = new Action({
        ros: this.ros,
        name: 'SegmentAndRecognize'
      });

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from SegmentAndRecognize:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', '');
          _this.set('recognized_objects', result.outputs);
        }
        else {
          _this.set('progress_update', 'Failed to identify objects');
          _this.set('recognized_objects', {});
        }
      });

      console.log('Calling SegmentAndRecognize');
      action.execute();
    },

    pickupObject: function(object_id) {
      this.set('progress_update', 'Picking up object ' + object_id);
      var action = new Action({
        ros: this.ros,
        name: 'PickupObject'
      });

      // Set the object to pick up
      action.inputs.object_id = object_id;
      action.inputs.arm = 'right';

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from PickupObject:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'Pickup successful');
        } else {
          _this.set('progress_update', 'Failed to pick up object ' + object_id);
        }
      });

      console.log('Sending action PickupObject with args', action.inputs);
      action.execute();
    },

    // ----------------------------------------------------------------------
    // Interactive markers commands

    interactiveGripper: function(action_string, arm, lift) {
      var action = new Action({
        ros: this.ros,
        name: 'InteractiveGripper'
      });

      // Set parameters
      action.inputs.action = action_string;
      action.inputs.arm = arm;
      action.inputs.lift = lift;

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from InteractiveGripper:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'InteractiveGripper motion successful');
        } else {
          _this.set('progress_update', 'InteractiveGripper failed: ' + result.outcome);
        }
      });

      console.log('Sending action InteractiveGripper with args', action.inputs);
      action.execute();
    },

    // ----------------------------------------------------------------------
    // Docking and undocking from a table

    // Meta: we should be using the CPU in the mid-tier to sequence smach actions together, rather than
    // doing it in the browser like I've done here
    dockWithTable: function() {
      // First move the arms aside
      this.set('progress_update', 'Untucking arms');

      var action = new Action({
        ros: this.ros,
        name: 'TuckArms'
      });
      action.inputs.tuck_left = false;
      action.inputs.tuck_right = false;

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from TuckArms:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'Arm movement successful');
          _this._dockWithTable2();
        } else {
          _this.set('progress_update', 'Failed to untuck arms');
        }
      });

      action.execute();

    },

    _dockWithTable2: function() {
      // Next move the arms to the side
      this.set('progress_update', 'Moving right arm to side');

      var action = new Action({
        ros: this.ros,
        name: 'MoveArmToJoint'
      });
      action.inputs.arm_angles = [-2.135, -0.02, -1.64, -2.07, -1.64, -1.680, 1.398];
      action.inputs.arm = 'right';

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from MoveArmToJoint:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'Arm movement successful');
          _this._dockWithTable3();
        } else {
          _this.set('progress_update', 'Failed to move right arm to side');
        }
      });
      action.execute();
    },

    _dockWithTable3: function() {
      // Next move the arms to the side
      this.set('progress_update', 'Moving left arm to side');

      var action = new Action({
        ros: this.ros,
        name: 'MoveArmToJoint'
      });
      action.inputs.arm_angles =  [ 2.135, -0.02,  1.64, -2.07,  1.64, -1.680, 1.398];
      action.inputs.arm = 'left';

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from MoveArmToJoint:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'Arm movement successful');
          _this._dockWithTable4();
        } else {
          _this.set('progress_update', 'Failed to move left arm to side');
        }
      });
      action.execute();
    },

    _dockWithTable4: function() {
      // Then MoveTorso all the awy up
      this.set('progress_update', 'Raising torso');

      var action = new Action({
        ros: this.ros,
        name: 'MoveTorso'
      });
      action.inputs.position = 0.295;

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from MoveTorso:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'Torso move successful');
          _this._dockWithTable5();
        } else {
          _this.set('progress_update', 'Failed to move torso');
        }
      });
      action.execute();
    },

    _dockWithTable5: function() {
      // Finally move forward a bit
      this.set('progress_update', 'Moving forwards');

      var action = new Action({
        ros: this.ros,
        name: 'NavigateToPose'
      });
      action.inputs.frame_id = '/base_footprint';
      action.inputs.x = 0.5;
      action.inputs.y = 0;
      action.inputs.theta = 0;
      action.inputs.collision_aware = false;

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from NavigateToPose:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'Moving forward successful');
          _this._dockWithTable6();
        } else {
          _this.set('progress_update', 'Failed to move torso');
        }
      });
      action.execute();
    },

    _dockWithTable6: function() {
      // Finally look down at the table
      this.set('progress_update', 'Looking down');

      var action = new Action({
        ros: this.ros,
        name: 'PointHead'
      });

      // Get notified when head movement finishes
      var _this = this;
      action.on('result', function(result) {
        _this.set('progress_update', 'Look forward ' + result.outcome);
        // Regardless of outcome, tell the UI to say that plugging is done
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'Finished docking with table');
        } else {
          _this.set('progress_update', 'Failed to look at table');
        }
      });

      action.inputs.target_frame   = 'torso_lift_link';
      action.inputs.target_x       = 1.0;
      action.inputs.target_y       = 0;
      action.inputs.target_z       = -0.4;
      action.inputs.pointing_frame = 'head_mount_kinect_rgb_optical_frame';
      action.inputs.pointing_x     = 0.0;
      action.inputs.pointing_y     = 0.0;
      action.inputs.pointing_z     = 1.0;
      action.execute();
      console.log('Calling PointHead action', action.inputs);
    },

    undockFromTable: function() {
      // Clear out the recognized objects since we'll be moving around and the data is stale
      this.set('recognized_objects', {});

      // First move backwards a bit
      this.set('progress_update', 'Moving backwards');

      var action = new Action({
        ros: this.ros,
        name: 'NavigateToPose'
      });
      action.inputs.frame_id = '/base_footprint';
      action.inputs.x = -0.5;
      action.inputs.y = 0;
      action.inputs.theta = 0;
      action.inputs.collision_aware = false;

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from NavigateToPose:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'Move backwards successful');
          _this._undockFromTable2();
        } else {
          _this.set('progress_update', 'Move backwards failed');
        }
      });
      action.execute();
    },

    _undockFromTable2: function() {
      // Then move the torso back down
      this.set('progress_update', 'Lowering torso');

      var action = new Action({
        ros: this.ros,
        name: 'MoveTorso'
      });
      action.inputs.position = 0.0125;

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from MoveTorso:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'Torso lowered successfully');
          _this._undockFromTable3();
        } else {
          _this.set('progress_update', 'Torso lowering failed');
        }
      });
      action.execute();
    },

    _undockFromTable3: function() {
      // Then tuck the arms
      this.set('progress_update', 'Tucking arms');

      var action = new Action({
        ros: this.ros,
        name: 'TuckArms'
      });
      action.inputs.tuck_left = true;
      action.inputs.tuck_right = true;

      var _this = this;
      action.on('result', function(result) {
        console.log('Result from TuckArms:', result);
        if (result.outcome === 'succeeded') {
          // It worked!
          _this.set('progress_update', 'Arms tucked successfully');
          _this._pointHeadForward(function() {
            _this._undockFromTable4();
          });
        } else {
          _this.set('progress_update', 'Failed to tuck arms');
        }
      });
      action.execute();
    },

    _undockFromTable4: function() {
      this.set('progress_update', 'Undocked from table');
    }

  });
});

function myDebugEvents(source, id, events) {
  for (var i=0; i < events.length; i++) {
    eval('var f = function(e){ console.log("["+id+".'+events[i]+'] received"); console.dir(arguments); };');
    source.on(events[i], f);
  }
}
