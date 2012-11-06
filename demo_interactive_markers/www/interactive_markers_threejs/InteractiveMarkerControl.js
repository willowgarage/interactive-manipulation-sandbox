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

THREE.InteractiveMarkerControl = function(parent, control) {
  THREE.Object3D.call(this);
  THREE.EventTarget.call(this);

  this.parent = parent;
  this.dragging = false;

  var that = this;

  var NONE = 0;
  var MENU = 1;
  var BUTTON = 2;
  var MOVE_AXIS = 3;
  var MOVE_PLANE = 4;
  var ROTATE_AXIS = 5;
  var MOVE_ROTATE = 6;

  var controlAxis = new THREE.Vector3(1, 0, 0);
  var controlOrientation = new THREE.Quaternion(control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w);

  // transform x axis into local frame
  controlOrientation.multiplyVector3(controlAxis);

  // determine mouse interaction
  switch(control.interaction_mode) {
    case MOVE_AXIS:
      this.addEventListener("mousemove", parent.moveAxis.bind(parent, that, controlAxis));
      break;
    case ROTATE_AXIS:
      this.addEventListener("mousemove", parent.rotateAxis.bind(parent, that, controlOrientation));
      break;
    case MOVE_PLANE:
      this.addEventListener("mousemove", parent.movePlane.bind(parent, that, controlAxis));
      break;
    default:
      break;
  }

  // install default listeners for highlighting / dragging
  function stopPropagation(event) {
    event.stopPropagation();
  }

  if (control.interaction_mode != NONE) {
    this.addEventListener('mousedown', parent.startDrag.bind(parent));
    this.addEventListener('mouseup', parent.stopDrag.bind(parent));
    this.addEventListener('mouseover', stopPropagation);
    this.addEventListener('mouseout', stopPropagation);
  }

  // define rotation behaviour
  var INHERIT = 0;
  var FIXED = 1;
  var VIEW_FACING = 2;

  var rotInv = new THREE.Quaternion();
  var posInv = parent.position.clone().multiplyScalar(-1);

  switch(control.orientation_mode) {
    case INHERIT:
      rotInv = parent.quaternion.clone().inverse();
      break;
    case FIXED:
      that.updateMatrixWorld = function(force) {
        //console.log("sdfsdf")
        that.useQuaternion = true;
        that.quaternion = that.parent.quaternion.clone().inverse();
        that.updateMatrix();
        that.matrixWorldNeedsUpdate = true;
        THREE.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
      }
      break;
    case VIEW_FACING:
      break;
    default:
      break;
  }

  // create visuals (markers)
  control.markers.forEach(function(markerMsg) {
    var markerHelper = new THREE.MarkerHelper(markerMsg);

    // convert position into my own local coordinate frame
    markerHelper.position.addSelf(posInv);
    rotInv.multiplyVector3(markerHelper.position);
    markerHelper.quaternion.multiply(rotInv, markerHelper.quaternion);
    markerHelper.updateMatrixWorld();

    that.add(markerHelper);
  });

}

THREE.InteractiveMarkerControl.prototype = Object.create(THREE.Object3D.prototype);

