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

THREE.InteractiveMarker = function(intMarkerMsg) {
  THREE.Object3D.call(this);
  THREE.EventTarget.call(this);

  this.name = intMarkerMsg.name;
  this.intMarkerMsg = intMarkerMsg;

  var that = this;

  this.onServerSetPose({pose:intMarkerMsg.pose});

  intMarkerMsg.controls.forEach(function(control) {
    that.add(new THREE.InteractiveMarkerControl(that, control));
  });
};

THREE.InteractiveMarker.prototype = Object.create(THREE.Object3D.prototype);

THREE.InteractiveMarker.prototype.moveAxis = function(axis, event3d) {
  if (this.dragging) {
    var translation, rotation = new THREE.Quaternion(), scale;
    this.matrixWorld.decompose(translation, rotation, scale);

    var axisRay = new THREE.Ray(this.dragStartPoint, rotation.multiplyVector3(axis.clone()));
    var t = INTERACT3D.closestAxisPoint(axisRay, event3d.camera, event3d.mousePos);

    var p = new THREE.Vector3;
    p.add(this.dragStartPosition, axisRay.direction.multiplyScalar(t));
    this.setPosition(p);
    event3d.stopPropagation();
  }
};

THREE.InteractiveMarker.prototype.startDrag = function(event3d) {
  console.log('start dragging');
  this.dragging = true;
  this.dragStartPoint = event3d.intersection.point.clone();
  this.dragStartPosition = this.position.clone();
  event3d.stopPropagation();
}

THREE.InteractiveMarker.prototype.stopDrag = function(event3d) {
  console.log('stop dragging');
  this.dragging = false;
  event3d.stopPropagation();
  this.onServerSetPose(this.bufferedPoseEvent);
  this.bufferedPoseEvent = undefined;
}

THREE.InteractiveMarker.prototype.setPosition = function(position) {
  this.position = position;
  this.dispatchEvent({
    type : "user_changed_pose",
    position: this.position,
    orientation: this.quaternion
  });
}

THREE.InteractiveMarker.prototype.onServerSetPose = function(event) {
  if (event === undefined) {
    return;
  }

  if (this.dragging) {
    this.bufferedPoseEvent = event;
    return;
  }

  var pose = event.pose;

  this.position.x = pose.position.x;
  this.position.y = pose.position.y;
  this.position.z = pose.position.z;

  this.useQuaternion = true;
  this.quaternion = new THREE.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

  this.updateMatrixWorld();
}
