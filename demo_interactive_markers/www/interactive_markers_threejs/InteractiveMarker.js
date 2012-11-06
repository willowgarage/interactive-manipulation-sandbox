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

  var that = this;

  this.name = intMarkerMsg.name;
  this.intMarkerMsg = intMarkerMsg;

  this.dragging = false;
  this.onServerSetPose({
    pose : intMarkerMsg.pose
  });

  this.dragStart = {
    position : new THREE.Vector3(),
    rotation : new THREE.Quaternion(),
    positionWorld : new THREE.Vector3(),
    rotationWorld : new THREE.Quaternion(),
    event3d : {}
  };

  intMarkerMsg.controls.forEach(function(control) {
    that.add(new THREE.InteractiveMarkerControl(that, control));
  });
};

THREE.InteractiveMarker.prototype = Object.create(THREE.Object3D.prototype);

THREE.InteractiveMarker.prototype.moveAxis = function(axis, event3d) {
  if (this.dragging) {
    // get move axis in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var axisWorld = this.dragStart.rotationWorld.clone().multiplyVector3(axis.clone());
    
    var axisRay = new THREE.Ray(originWorld, axisWorld);
    
    // find closest point to mouse on axis
    var t = INTERACT3D.closestAxisPoint(axisRay, event3d.camera, event3d.mousePos);

    // offset from drag start position
    var p = new THREE.Vector3;
    p.add(this.dragStart.position, this.dragStart.rotation.multiplyVector3(axis.clone()).multiplyScalar(t));
    this.setPosition(p);
    
    event3d.stopPropagation();
  }
};

THREE.InteractiveMarker.prototype.movePlane = function(normal, event3d) {
  if (this.dragging) {
    // get plane params in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var normalWorld = this.dragStart.rotationWorld.multiplyVector3(normal.clone());

    // intersect mouse ray with plane
    var intersection = INTERACT3D.intersectPlane(event3d.mouseRay, originWorld, normalWorld);

    // offset from drag start position
    var p = new THREE.Vector3;
    p.sub(intersection, originWorld);
    p.addSelf(this.dragStart.positionWorld);
    this.setPosition(p);
    event3d.stopPropagation();
  }
};

THREE.InteractiveMarker.prototype.rotateAxis = function(orientation, event3d) {
  if (this.dragging) {

    var normal = orientation.multiplyVector3(new THREE.Vector3(1, 0, 0));

    // get plane params in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var normalWorld = this.dragStart.rotationWorld.multiplyVector3(normal);

    // intersect mouse ray with plane
    var intersection = INTERACT3D.intersectPlane(event3d.mouseRay, originWorld, normalWorld);
    
    // offset local origin to lie on intersection plane
    var normalRay = new THREE.Ray( this.dragStart.positionWorld, normalWorld );
    var rotOrigin = INTERACT3D.intersectPlane(normalRay, originWorld, normalWorld);

    // rotates from world to plane coords
    var orientationWorld = this.dragStart.rotationWorld.clone().multiplySelf(orientation);
    var orientationWorldInv = orientationWorld.clone().inverse();
    
    // rotate original and current intersection into local coords
    intersection.subSelf( rotOrigin );
    orientationWorldInv.multiplyVector3(intersection);

    var origIntersection = this.dragStart.event3d.intersection.point.clone();
    origIntersection.subSelf( rotOrigin );
    orientationWorldInv.multiplyVector3(origIntersection);
    
    // compute relative 2d angle
    var a1 = Math.atan2(intersection.y,intersection.z);
    var a2 = Math.atan2(origIntersection.y,origIntersection.z);
    var a = a2 - a1;
    
    var rot = new THREE.Quaternion();
    rot.setFromAxisAngle( normal, a );
    
    // rotate
//    this.setOrientation( rot.multiplySelf(this.dragStart.rotationWorld) );
    this.setOrientation( rot.multiplySelf(this.dragStart.rotationWorld) );
    
    // offset from drag start position
    event3d.stopPropagation();
  }
};

THREE.InteractiveMarker.prototype.startDrag = function(event3d) {
  console.log('start dragging');
  event3d.stopPropagation();
  this.dragging = true;
  this.updateMatrixWorld(true);
  var scale = new THREE.Vector3();
  this.matrixWorld.decompose(this.dragStart.positionWorld, this.dragStart.rotationWorld, scale);
  this.dragStart.position = this.position.clone();
  this.dragStart.rotation = this.quaternion.clone();
  this.dragStart.event3d = event3d;
  console.log(this.dragStart.rotationWorld);
}

THREE.InteractiveMarker.prototype.stopDrag = function(event3d) {
  console.log('stop dragging');
  event3d.stopPropagation();
  this.dragging = false;
  this.dragStart.event3d = {};
  this.onServerSetPose(this.bufferedPoseEvent);
  this.bufferedPoseEvent = undefined;
}

THREE.InteractiveMarker.prototype.setPosition = function(position) {
  this.position = position;
  this.dispatchEvent({
    type : "user_changed_pose",
    position : this.position,
    orientation : this.quaternion
  });
}

THREE.InteractiveMarker.prototype.setOrientation = function(orientation) {
  orientation.normalize();
  this.quaternion = orientation;
  this.dispatchEvent({
    type : "user_changed_pose",
    position : this.position,
    orientation : this.quaternion
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

  this.updateMatrixWorld(true);
}
