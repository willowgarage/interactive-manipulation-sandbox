/**
 * @author dgossow@willowgarage.com
 */

THREE.InteractiveMarkerHelper = function ( intMarkerMsg ) 
{
	THREE.Object3D.call( this );

  intMarkerObj = new THREE.Object3D;

  intMarkerObj.position.x = intMarkerMsg.pose.position.x;
  intMarkerObj.position.y = intMarkerMsg.pose.position.y;
  intMarkerObj.position.z = intMarkerMsg.pose.position.z;
  
  intMarkerObj.useQuaternion = true;
  intMarkerObj.quaternion = new THREE.Quaternion(
    intMarkerMsg.pose.orientation.x,
    intMarkerMsg.pose.orientation.y,
    intMarkerMsg.pose.orientation.z
  );

  intMarkerMsg.controls.forEach(function(control) 
  {
    control.markers.forEach(function(markerMsg)
    {
      var markerHelper = new THREE.MarkerHelper(markerMsg);
      intMarkerObj.add(markerHelper);
    });
  });
  
  this.add( intMarkerObj );
};

THREE.InteractiveMarkerHelper.prototype = Object.create( THREE.Object3D.prototype );
