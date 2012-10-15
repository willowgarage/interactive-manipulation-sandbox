/**
 * @author dgossow@willowgarage.com
 */

THREE.InteractiveMarkerHelper = function ( intMarkerMsg ) 
{
  THREE.Object3D.call( this );
  
  this.name = intMarkerMsg.name;

  var that = this;

  this.position.x = intMarkerMsg.pose.position.x;
  this.position.y = intMarkerMsg.pose.position.y;
  this.position.z = intMarkerMsg.pose.position.z;
  
  this.useQuaternion = true;
  this.quaternion = new THREE.Quaternion(
    intMarkerMsg.pose.orientation.x,
    intMarkerMsg.pose.orientation.y,
    intMarkerMsg.pose.orientation.z,
    intMarkerMsg.pose.orientation.w
  );
  
  this.updateMatrixWorld();

  intMarkerMsg.controls.forEach(function(control) 
  {
    control.markers.forEach(function(markerMsg)
    {
      var markerHelper = new THREE.MarkerHelper(markerMsg);

      //var m_inv = that.matrixWorld.clone();
      //m_inv.getInverse();
      
      // convert position into my own local coordinate frame
      markerHelper.position = that.worldToLocal( markerHelper.position );
      //?? markerHelper.quaternion = that.worldToLocal( markerHelper.position );
      //console.log("adding ",markerHelper);
      that.add(markerHelper);
    });
  });
};

THREE.InteractiveMarkerHelper.prototype = Object.create( THREE.Object3D.prototype );

//THREE.Camera.prototype.lookAt = function ( vector ) {
//  this.matrix.lookAt( this.position, vector, this.up );


THREE.InteractiveMarkerHelper.prototype.setPose = function ( pose ) 
{
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
}
