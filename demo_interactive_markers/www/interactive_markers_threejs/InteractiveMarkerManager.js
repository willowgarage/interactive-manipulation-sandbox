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

THREE.InteractiveMarkerManager = function ( scene, intMarkerClient ) 
{
  this.scene = scene;
  this.root = new THREE.Object3D();
  scene.add(this.root);
  
  var that=this;
  
  intMarkerClient.on('created_marker', this.addMarker.bind(this));
  intMarkerClient.on('deleted_marker', this.eraseMarker.bind(this));
};

THREE.InteractiveMarkerManager.prototype = {
  
  addMarker: function(marker) {
    var markerView = new THREE.InteractiveMarker(marker, this.feedbackTopic);
    this.root.add(markerView);
    marker.on('server_updated_pose', function(pose) {
      markerView.onServerSetPose({
        pose : pose
      });
    });
    markerView.addEventListener('user_changed_pose', function() {
      var pose = {
        position: markerView.position,
        orientation: markerView.orientation
      };
      marker.setPoseFromClient(pose);
    });
    markerView.addEventListener('user_mouse_down',marker.onMouseDown.bind(marker));
    markerView.addEventListener('user_mouse_up', marker.onMouseUp.bind(marker));
    markerView.addEventListener('user_clicked_button', marker.onButtonClick.bind(marker));
  },
  
  eraseMarker: function(event) {
    this.root.remove(this.root.getChildByName(event.name));
  },
}
