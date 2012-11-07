/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

var interactive_markers = interactive_markers || {};

interactive_markers.InteractiveMarker = function(clientId, intMarkerMsg, feedbackTopic) {
  var that = this;
  this.feedbackTopic = feedbackTopic;
  this.position = this.copyXyz(intMarkerMsg.pose.position);
  this.orientation = this.copyXyzw(intMarkerMsg.pose.orientation);
  this.intMarkerMsg = intMarkerMsg;
  this.clientId = clientId;

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
    pose : poseMsg
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

interactive_markers.InteractiveMarker.prototype.onMouseDown = function(event) {
  this.sendFeedback(4, event.clickPosition);
}

interactive_markers.InteractiveMarker.prototype.onMouseUp = function(event) {
  this.sendFeedback(5, event.clickPosition);
}

interactive_markers.InteractiveMarker.prototype.sendFeedback = function(eventType, clickPosition) {

  var clickPosition = clickPosition || {
    x : 0,
    y : 0,
    z : 0
  };

  var f = {
    header : this.intMarkerMsg.header,
    client_id : this.clientId,
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
