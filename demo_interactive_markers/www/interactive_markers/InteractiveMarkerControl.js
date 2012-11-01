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

THREE.InteractiveMarker.ORIENTATION_MODE = {
    INHERIT: 0, FIXED: 1, VIEW_FACING: 2};

THREE.InteractiveMarkerControl = function ( parent, control ) 
{
  THREE.Object3D.call( this );
  
  this.parent = parent;
  this.dragging = false;
  
  var that = this;

  var MODE = {
    NONE: 0, MENU: 1, BUTTON: 2, MOVE_AXIS: 3, 
    MOVE_PLANE: 4, ROTATE_AXIS:5, MOVE_ROTATE: 6};

  // determine mouse interaction
  switch(control.interaction_mode)
  {
  case MODE.MOVE_AXIS:
    var controlAxis = new THREE.Vector3( 1, 0, 0 );
    var controlOrientation = new THREE.Quaternion( control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w );

    // transform x axis into local frame
    controlOrientation.multiplyVector3( controlAxis );
    
    this.onmousemove = parent.moveAxis.bind( parent, controlAxis );
    break;
  }
  
  // create visuals (markers)
  control.markers.forEach(function(markerMsg)
  {
    var markerHelper = new THREE.MarkerHelper(markerMsg);

    // convert position into my own local coordinate frame
    markerHelper.position = that.worldToLocal( markerHelper.position );
    markerHelper.quaternion.multiply( that.quaternion.clone().inverse(), markerHelper.quaternion );
    markerHelper.updateMatrixWorld();
    
    that.add(markerHelper);
  });
  


/*
  var that = this;
  this.onmouseover = function( event )
  {
    console.log( 'onmouseover' );
    that.shaft.material = that.hoverMaterial;
    that.cone.material = that.hoverMaterial;
  }
  this.onmouseout = function( event )
  {
    console.log( 'onmouseout' );
    that.shaft.material = that.normalMaterial;
    that.cone.material = that.normalMaterial;
  }
  this.onmousedown = function( event )
  {
    console.log( 'onmousedown' );
    that.shaft.material = that.activeMaterial;
    that.cone.material = that.activeMaterial;
  }
  this.onmouseup = function( event )
  {
    console.log( 'onmouseup' );
    that.shaft.material = that.hoverMaterial;
    that.cone.material = that.hoverMaterial;

    this.dragging = false;
  }

  this.normalMaterial = options.material || new THREE.MeshBasicMaterial();
  
  this.hoverMaterial = this.normalMaterial.clone();
  this.hoverMaterial.color.r = 1;
  this.hoverMaterial.color.g = 1;
  this.hoverMaterial.color.b = 1;

  this.activeMaterial = this.normalMaterial.clone();
  this.activeMaterial.emissive.r *= 0.3;
  this.activeMaterial.emissive.g *= 0.3;
  this.activeMaterial.emissive.b *= 0.3;
  this.activeMaterial.opacity = 1.0;
  this.activeMaterial.transparent = false;
  this.activeMaterial.depthWrite = true;
  this.activeMaterial.blending = THREE.NormalBlending;

  this.material = this.normalMaterial;
*/
  
}

THREE.InteractiveMarkerControl.prototype = Object.create( THREE.Object3D.prototype );

