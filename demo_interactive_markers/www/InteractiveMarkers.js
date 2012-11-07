(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['eventemitter2'], factory);
  }
  else {
    root.InteractiveMarkers = factory(root.EventEmitter2);
  }
}(this, function (EventEmitter2) {

  var InteractiveMarkers = {};

  var Client = InteractiveMarkers.Client = function(ros) {
    this.ros = ros;
    this.interactiveMarkers = {};
  };
  Client.prototype.__proto__ = EventEmitter2.prototype;

  Client.prototype.subscribe = function(topicName) {
    this.markerUpdateTopic = new this.ros.Topic({
      name : topicName + '/tunneled',
      messageType : 'visualization_msgs/InteractiveMarkerUpdate'
    });
    this.markerUpdateTopic.subscribe(this.markerUpdate.bind(this));

    this.feedbackTopic = new this.ros.Topic({
      name : topicName + '/feedback',
      messageType : 'visualization_msgs/InteractiveMarkerFeedback'
    });
    this.feedbackTopic.advertise();
  };

  Client.prototype.markerUpdate = function(message) {
    var that = this;

    // Deletes markers
    message.erases.forEach(function(name) {
      var marker = that.interactiveMarkers[name];
      delete that.interactiveMarkers[name];
      that.emit('deleted_marker', marker);
    });

    // Updates marker poses
    message.poses.forEach(function(poseMessage) {
      var marker = that.interactiveMarkers[poseMessage.name];
      if (marker) {
        marker.setPoseFromServer(poseMessage.pose);
      }
    });

    // Adds new markers
    message.markers.forEach(function(markerMessage) {
      var oldMarker = that.interactiveMarkers[markerMessage.name];
      if (oldMarker) {
        that.emit('deleted_marker', oldMarker);
        delete that.interactiveMarkers[markerMessage.name];
      }

      var marker = new InteractiveMarkers.InteractiveMarker(markerMessage, that.feedbackTopic);
      that.interactiveMarkers[markerMessage.name] = marker;
      that.emit('created_marker', marker);
    });
  };

  var InteractiveMarker = InteractiveMarkers.InteractiveMarker = function(options, feedbackTopic) {
    this.feedbackTopic = feedbackTopic;
    this.pose     = options.pose;
    this.name     = options.name;
    this.header   = options.header;
    this.controls = options.controls;
    this.on('client_updated_pose', this.sendFeedback.bind(this, 1));
  };
  InteractiveMarker.prototype.__proto__ = EventEmitter2.prototype;

  InteractiveMarker.prototype.setPoseFromServer = function(pose) {
    this.pose = pose;
    this.emit('server_updated_pose', pose);
  };

  InteractiveMarker.prototype.setPoseFromClient = function(pose) {
    this.pose = pose;
    this.emit('client_updated_pose', pose);
  };

  InteractiveMarker.prototype.onButtonClick = function(event) {
    this.sendFeedback(3, event.clickPosition);
  };

  InteractiveMarker.prototype.onMouseDown = function(event) {
    this.sendFeedback(4, event.clickPosition);
  }

  InteractiveMarker.prototype.onMouseUp = function(event) {
    this.sendFeedback(5, event.clickPosition);
  }

  /*
    var KEEP_ALIVE = 0;
    var POSE_UPDATE = 1;
    var MENU_SELECT = 2;
    var BUTTON_CLICK = 3;
    var MOUSE_DOWN = 4;
    var MOUSE_UP = 5;
  */
  InteractiveMarker.prototype.sendFeedback = function(eventType, clickPosition) {
    var clickPosition = clickPosition || {
      x : 0,
      y : 0,
      z : 0
    };

    var feedback = {
      header       : this.header,
      client_id    : '',
      marker_name  : this.name,
      control_name : '',
      event_type   : eventType,
      pose         : this.pose,
      mouse_point  : clickPosition
    }

    this.feedbackTopic.publish(feedback);
  };

  return InteractiveMarkers;
}));
