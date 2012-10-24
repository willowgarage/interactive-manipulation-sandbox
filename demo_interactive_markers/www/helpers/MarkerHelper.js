/**
 * @author dgossow@willowgarage.com
 */

THREE.MarkerHelper = function ( markerMsg ) 
{
  THREE.Object3D.call( this );
  var that = this;

  function makeColorMaterial(r, g, b, a) {
    var color = new THREE.Color();
    color.setRGB(r, g, b);
    var color2 = new THREE.Color();
    var transparent = (a <= 0.99);
    color2.setRGB(r*0.3, g*0.3, b*0.3);
    return new THREE.MeshLambertMaterial({
    //return new THREE.MeshPhongMaterial({
      color : color.getHex(),
      emissive: color.getHex(),
      opacity: a,
      transparent: transparent,
      depthWrite: !transparent,
      blending: transparent ? THREE.AdditiveBlending : THREE.NormalBlending,
    });
  }
  
  function addMesh( geom, mat )
  {
    var mesh = new THREE.Mesh(geom, mat);
    /*
    //console.log(markerHelper);
    mesh.on('mouseover', function(arguments){ 
      console.log( "intersect: ", arguments.intersect.point );
      that.scale.x = 1.2;
      that.scale.y = 1.2;
      that.scale.z = 1.2;
    });
    mesh.on('mouseout', function(arguments){
      that.scale.x = 1.0;
      that.scale.y = 1.0;
      that.scale.z = 1.0;
    });
    */
    that.add( mesh );
  }
  
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

  this.setPose( markerMsg.pose );

  var colorMaterial = makeColorMaterial( markerMsg.color.r, markerMsg.color.g,
      markerMsg.color.b, markerMsg.color.a );

  switch( markerMsg.type ) {
  case 0: // ARROW
    var len = markerMsg.scale.x;
    var headLen = len * 0.23;
    var headR = markerMsg.scale.y;
    var shaftR = headR * 0.5;
    if ( markerMsg.points.length == 2 )
    {
      var p1 = new THREE.Vector3( markerMsg.points[0].x, markerMsg.points[0].y, markerMsg.points[0].z );
      var p2 = new THREE.Vector3( markerMsg.points[1].x, markerMsg.points[1].y, markerMsg.points[1].z );
      var dir = p1.clone().negate().addSelf( p2 ); // dir = p2 - p1;
      len = dir.length();
      headR = markerMsg.scale.y;
      shaftR = markerMsg.scale.x;

      if ( markerMsg.scale.z != 0.0 )
      {
        headLen = markerMsg.scale.z;
      }
    }

    var arrow = new THREE.ArrowMarkerHelper({
      dir: dir,
      origin: p1,
      length: len,
      headLength: headLen,
      shaftDiameter: shaftR,
      headDiameter: headR,
      material: colorMaterial 
      });
    this.add(arrow);

    break;
  case 1: // CUBE
    var geom = new THREE.CubeGeometry(markerMsg.scale.x, markerMsg.scale.y, markerMsg.scale.z);
    addMesh(geom, colorMaterial);
    break;
  case 11: // TRIANGLE_LIST
    this.add( new THREE.TriangleListMarkerHelper( colorMaterial, markerMsg.points ) );
    break;
  default:
    geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    addMesh(geom, colorMaterial);
    break;
  }

  geom = new THREE.CubeGeometry(0.1,0.1,0.1);
  addMesh(geom, new THREE.MeshBasicMaterial);

};

THREE.MarkerHelper.prototype = Object.create( THREE.Object3D.prototype );

THREE.MarkerHelper.prototype.setPose = function ( pose ) 
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
  this.quaternion.normalize();
  
  this.updateMatrixWorld();
}
