/**
 * @author dgossow@willowgarage.com
 */

function makeColorMaterial(r, g, b) {
  var color = new THREE.Color;
  color.setRGB(r, g, b);
  return new THREE.MeshLambertMaterial({
  //return new THREE.MeshPhongMaterial({
    color : color.getHex()
  });
}

THREE.MarkerHelper = function ( markerMsg ) 
{
  var geom;

  /*
   uint8 ARROW=0
   uint8 CUBE=1
   uint8 SPHERE=2
   uint8 CYLINDER=3
   uint8 LINE_STRIP=4
   uint8 LINE_LIST=5
   uint8 CUBE_LIST=6
   uint8 SPHERE_LIST=7
   uint8 POINTS=8
   uint8 TEXT_VIEW_FACING=9
   uint8 MESH_RESOURCE=10
   uint8 TRIANGLE_LIST=11
  */

  switch( markerMsg.type ) {
    case 1:
      geom = new THREE.CubeGeometry(markerMsg.scale.x, markerMsg.scale.y, markerMsg.scale.z);
      break;
    default:
      geom = new THREE.CubeGeometry(0.01,0.01,0.01);
    //geom = new THREE.CubeGeometry(1,1,1);
  }

  THREE.Object3D.call( this );
  
  var colorMaterial = makeColorMaterial(
      markerMsg.color.r, 
      markerMsg.color.b, 
      markerMsg.color.b);
      
  this.markerMesh = new THREE.Mesh(geom, colorMaterial);
  
  this.markerMesh.position.x = markerMsg.pose.position.x;
  this.markerMesh.position.y = markerMsg.pose.position.y;
  this.markerMesh.position.z = markerMsg.pose.position.z;
  
  this.markerMesh.useQuaternion = true;
  this.markerMesh.quaternion = new THREE.Quaternion(
    markerMsg.pose.orientation.x,
    markerMsg.pose.orientation.y,
    markerMsg.pose.orientation.z,
    markerMsg.pose.orientation.w
  );
  
  this.add( this.markerMesh );
	
};

THREE.MarkerHelper.prototype = Object.create( THREE.Object3D.prototype );
