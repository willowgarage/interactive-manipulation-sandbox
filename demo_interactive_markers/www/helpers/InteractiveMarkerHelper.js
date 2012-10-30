/**
 * @author dgossow@willowgarage.com
 */

THREE.InteractiveMarkerHelper = function ( intMarkerMsg ) 
{
  THREE.Object3D.call( this );
  
  this.name = intMarkerMsg.name;

  var that = this;
  
  this.setPose( intMarkerMsg.pose );

  //console.log(intMarkerMsg);

  intMarkerMsg.controls.forEach(function(control) 
  {
    var MODE = THREE.InteractiveMarkerHelper.INTERACTION_MODE;

    switch(control.interaction_mode)
    {
    case MODE.MOVE_AXIS:
      var controlAxis = new THREE.Vector3( 1, 0, 0 );
      var controlOrientation = new THREE.Quaternion( control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w );

      // transform x axis into local frame
      controlOrientation.multiplyVector3( controlAxis );
      
      var onmousemove = THREE.InteractiveMarkerHelper.prototype.moveAxis.bind( that, controlAxis );
      break;
    }
      
    control.markers.forEach(function(markerMsg)
    {
      var markerHelper = new THREE.MarkerHelper(markerMsg);

      // convert position into my own local coordinate frame
      markerHelper.position = that.worldToLocal( markerHelper.position );
      markerHelper.quaternion.multiply( that.quaternion.clone().inverse(), markerHelper.quaternion );
      markerHelper.updateMatrixWorld();
      
      //console.log(onmousemove);
      markerHelper.onmousemove = onmousemove;
      
      that.add(markerHelper);
    });
    
  });
  
};

THREE.InteractiveMarkerHelper.ORIENTATION_MODE = {
    INHERIT: 0, FIXED: 1, VIEW_FACING: 2};

THREE.InteractiveMarkerHelper.INTERACTION_MODE = {
    NONE: 0, MENU: 1, BUTTON: 2, MOVE_AXIS: 3, 
    MOVE_PLANE: 4, ROTATE_AXIS:5, MOVE_ROTATE: 6};



THREE.InteractiveMarkerHelper.prototype = Object.create( THREE.Object3D.prototype );

function findClosestPoint( target_ray, mouse_ray )
{
  // Find the closest point on target_ray to any point on mouse_ray.
  // Math taken from http://paulbourke.net/geometry/lineline3d/
  // line P1->P2 is target_ray
  // line P3->P4 is mouse_ray

  var v13 = new THREE.Vector3;
  v13.sub(target_ray.origin, mouse_ray.origin );
  var v43 = mouse_ray.direction.clone();
  var v21 = target_ray.direction.clone();
  var d1343 = v13.dot( v43 );
  var d4321 = v43.dot( v21 );
  var d1321 = v13.dot( v21 );
  var d4343 = v43.dot( v43 );
  var d2121 = v21.dot( v21 );

  var denom = d2121 * d4343 - d4321 * d4321;
  if( Math.abs( denom ) <= 0.0001 )
  {
    return undefined;
  }
  var numer = d1343 * d4321 - d1321 * d4343;

  var mua = numer / denom;
  return mua;
}

function closestAxisPoint( axisRay, camera, mousePos )
{
  // project axis onto screen
  var projector = new THREE.Projector();
  
  var o = axisRay.origin.clone();
  projector.projectVector(o,camera);
  
  var o2 = axisRay.direction.clone().addSelf( axisRay.origin );
  projector.projectVector(o2,camera);
  
  // d is the axis vector in screen space
  var d = o2.clone().subSelf( o ); // d = o2-o;
  
  // t is the 2d ray param of perpendicular projection
  // of mousePos onto o
  var tmp = new THREE.Vector2;
  var t = tmp.sub(mousePos,o).dot(d) / d.dot(d); // t = (mousePos - o) * d / (d*d);
  
  // mp is the final 2d-projected mouse pos
  var mp = new THREE.Vector2;
  mp.add( o, d.clone().multiplyScalar(t) ); // mp = o + d*t;
  
  // go back to 3d by shooting a ray
  var vector = new THREE.Vector3(mp.x, mp.y, 0.5);
  projector.unprojectVector(vector, camera);
  var mpRay = new THREE.Ray(camera.position, vector.subSelf(camera.position).normalize());
  var mua = findClosestPoint( axisRay, mpRay, mua );

  return mua;
}

THREE.InteractiveMarkerHelper.prototype.moveAxis = function( axis, event ) {
  if( this.dragging ) {
    var translation, rotation=new THREE.Quaternion(), scale;
    this.matrixWorld.decompose(translation, rotation, scale);
    
    var axisRay = new THREE.Ray( this.dragStartPoint, rotation.multiplyVector3(axis.clone()) );
    var t = closestAxisPoint(axisRay,event.camera,event.mousePos);
    
    if ( t ) {
      this.position.add(this.dragStartPosition, axisRay.direction.multiplyScalar(t));
    }
  }
};


THREE.InteractiveMarkerHelper.prototype.onmousedown = function( event ) {
  console.log( 'start dragging' );
  this.dragging = true;
  this.dragStartPoint = event.intersectionPoint.clone();
  this.dragStartPosition = this.position.clone();
  event.stopPropagation();
}

THREE.InteractiveMarkerHelper.prototype.onmouseup = function( event ) {
  console.log( 'stop dragging' );
  this.dragging = false;
  event.stopPropagation();
}

THREE.InteractiveMarkerHelper.prototype.onmousemove = function ( event ) {
  if( this.dragging ) {
    console.log( 'dragging' );
    event.stopPropagation();
  }
}

THREE.InteractiveMarkerHelper.prototype.setPose = function ( pose ) {
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
