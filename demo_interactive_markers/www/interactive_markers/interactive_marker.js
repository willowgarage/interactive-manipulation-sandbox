interactive_markers = interactive_markers || {};

interactive_markers.InteractiveMarker = function(intMarkerMsg, feedbackTopic) {
  var that = this;
  this.feedbackTopic = feedbackTopic;
  this.position = this.copyXyz(intMarkerMsg.pose.position);
  this.orientation = this.copyXyzw(intMarkerMsg.pose.orientation);
  this.intMarkerMsg = intMarkerMsg;

  //todo: use other event dispatcher (?)
  THREE.EventTarget.apply(this);

  /*
   var KEEP_ALIVE = 0;
   var POSE_UPDATE = 1;
   var MENU_SELECT = 2;
   var BUTTON_CLICK = 3;
   var MOUSE_DOWN = 4;
   var MOUSE_UP = 5;
   */
}

interactive_markers.InteractiveMarker.prototype.copyXyz = function(src) {
  return {
    x : src.x,
    y : src.y,
    z : src.z
  };
}

interactive_markers.InteractiveMarker.prototype.copyXyzw = function(src) {
  return {
    x : src.x,
    y : src.y,
    z : src.z,
    w : src.w
  };
}

interactive_markers.InteractiveMarker.prototype.serverSetPose = function(poseMsg) {
  this.position = this.copyXyz(poseMsg.position);
  this.orientation = this.copyXyzw(poseMsg.orientation);
  this.dispatchEvent({
    type : "server_changed_pose",
    pose: poseMsg
  });
}

interactive_markers.InteractiveMarker.prototype.onUserSetPose = function(event) {
  this.position = this.copyXyz(event.position);
  this.orientation = this.copyXyzw(event.orientation);
  this.sendFeedback(1);
}

interactive_markers.InteractiveMarker.prototype.onButtonClick = function(event) {
  this.sendFeedback(3, event.clickPosition);
}

interactive_markers.InteractiveMarker.prototype.sendFeedback = function(eventType, clickPosition) {

  var clickPosition = clickPosition || {
    x : 0,
    y : 0,
    z : 0
  };

  var f = {
    header : this.intMarkerMsg.header,
    client_id : "",
    marker_name : this.intMarkerMsg.name,
    control_name : "",
    event_type : eventType,
    pose : {
      position : this.copyXyz(this.position),
      orientation : this.copyXyzw(this.orientation)
    },
    mouse_point : clickPosition
  }

  this.feedbackTopic.publish(f);
}
