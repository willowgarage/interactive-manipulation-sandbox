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

THREE.InteractiveMarker = function ( intMarkerMsg, feedbackTopic ) 
{
  THREE.Object3D.call( this );
  
  this.name = intMarkerMsg.name;
  this.feedbackTopic = feedbackTopic;
  this.intMarkerMsg = intMarkerMsg;

  var that = this;
  
  this.setPose( intMarkerMsg.pose );

  //console.log(intMarkerMsg);

  intMarkerMsg.controls.forEach(function(control) 
  {
    that.add( new THREE.InteractiveMarkerControl( that, control ) );
  });
};

THREE.InteractiveMarker.prototype = Object.create( THREE.Object3D.prototype );

THREE.InteractiveMarker.prototype.moveAxis = function( axis, event ) {
  if( this.dragging ) {
    var translation, rotation=new THREE.Quaternion(), scale;
    this.matrixWorld.decompose(translation, rotation, scale);
    
    var axisRay = new THREE.Ray( this.dragStartPoint, rotation.multiplyVector3(axis.clone()) );
    var t = INTERACT3D.closestAxisPoint(axisRay,event.camera,event.mousePos);
    
    var p = new THREE.Vector3;
    p.add(this.dragStartPosition, axisRay.direction.multiplyScalar(t));
    this.setPosition(p);   
  }
};

THREE.InteractiveMarker.prototype.setPosition = function( position ) {
  this.position = position;
  this.sendFeedback();
}

THREE.InteractiveMarker.prototype.sendFeedback = function() {
  var f = {
    header: this.intMarkerMsg.header,
    client_id: "",
    marker_name: this.intMarkerMsg.name,
    control_name: "",
    event_type: 1,
    pose: {
      position: { 
        x: this.position.x,
        y: this.position.y,
        z: this.position.z },
      orientation: {
        x: this.quaternion.x,
        y: this.quaternion.y,
        z: this.quaternion.z,
        w: this.quaternion.w }
      },
    mouse_point: { 
      x: this.dragStartPoint.x,
      y: this.dragStartPoint.y,
      z: this.dragStartPoint.z }
  }
  
  this.feedbackTopic.publish(f);
}

THREE.InteractiveMarker.prototype.onmousedown = function( event ) {
  console.log( 'start dragging' );
  this.dragging = true;
  this.dragStartPoint = event.intersectionPoint.clone();
  this.dragStartPosition = this.position.clone();
  event.stopPropagation();
}

THREE.InteractiveMarker.prototype.onmouseup = function( event ) {
  console.log( 'stop dragging' );
  this.dragging = false;
  event.stopPropagation();
  this.setPose(this.bufferedPose);
  this.bufferedPose = undefined;
}

THREE.InteractiveMarker.prototype.onmousemove = function ( event ) {
  if( this.dragging ) {
    console.log( 'dragging' );
    event.stopPropagation();
  }
}

THREE.InteractiveMarker.prototype.setPose = function ( pose ) {
  if ( pose === undefined ) {
    return;
  }
  
  if ( this.dragging ) {
    this.bufferedPose = pose;
    return;
  }
  
  this.position.x = pose.position.x;
  this.position.y = pose.position.y;
  this.position.z = pose.position.z;

  this.useQuaternion = true;
  this.quaternion = new THREE.Quaternion(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w
  );

  this.updateMatrixWorld();
}
