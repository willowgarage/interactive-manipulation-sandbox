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

interactive_markers.Client = function(ros) {

  this.ros = ros;
  this.interactiveMarkers = {};
  
  THREE.EventTarget.call(this);
};

interactive_markers.Client.prototype = Object.create(THREE.Object3D.prototype);

interactive_markers.Client.prototype.subscribe = function(topicNS) {
  var ros = this.ros;

  this.topic = new ros.Topic({
    name : topicNS + '/tunneled',
    messageType : 'visualization_msgs/InteractiveMarkerUpdate'
  });
  this.topic.subscribe(this.processUpdate.bind(this));

  this.feedbackTopic = new ros.Topic({
    name : topicNS + '/feedback',
    messageType : 'visualization_msgs/InteractiveMarkerFeedback'
  });
  this.feedbackTopic.advertise();
}

interactive_markers.Client.prototype.processUpdate = function(message) {

  var that = this;

  // delete markers
  message.erases.forEach(function(name) {
    that.interactiveMarkers[name] = undefined;
    that.dispatchEvent({type:'erased_marker', name:name});
  });

  // update poses
  message.poses.forEach(function(poseMsg) {
    if (that.interactiveMarkers[poseMsg.name] != undefined) {
      that.interactiveMarkers[poseMsg.name].serverSetPose(poseMsg.pose);
    }
  });

  // add new markers
  message.markers.forEach(function(intMarkerMsg) {
    if (that.interactiveMarkers[intMarkerMsg.name] != undefined) {
    that.dispatchEvent({type:'erased_marker', intMarkerModel:intMarkerModel});
    }
    intMarkerModel = new interactive_markers.InteractiveMarker(intMarkerMsg, that.feedbackTopic);
    that.interactiveMarkers[intMarkerMsg.name] = intMarkerModel;
    that.dispatchEvent({type:'added_marker', intMarkerModel:intMarkerModel});
  });
};
