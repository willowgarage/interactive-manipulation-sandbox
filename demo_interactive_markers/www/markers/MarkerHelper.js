/**
 * @author dgossow@willowgarage.com
 */

THREE.MarkerHelper = function(markerMsg) {
  THREE.Object3D.call(this);

  var that = this;
  var geom = null;

  function makeColorMaterial(r, g, b, a) {
    var color = new THREE.Color();
    color.setRGB(r, g, b);
    if (a <= 0.99) {
      return new THREE.MeshBasicMaterial({
        color : color.getHex(),
        opacity : a+0.1,
        transparent : true,
        depthWrite : true,
        blendSrc : THREE.SrcAlphaFactor,
        blendDst : THREE.OneMinusSrcAlphaFactor,
        blendEquation : THREE.ReverseSubtractEquation,
        blending : THREE.NormalBlending,
      });
    } else {
      return new THREE.MeshLambertMaterial({
        color : color.getHex(),
        opacity : a,
        blending : THREE.NormalBlending,
      });
    }
  }

  function addMesh(geom, mat) {
    var mesh = new THREE.Mesh(geom, mat);
    that.add(mesh);
  }

  function pointMsgToVector3(msg) {
    return new THREE.Vector3(msg.x, msg.y, msg.z);
  }

  var ARROW = 0;
  var CUBE = 1;
  var SPHERE = 2
  var CYLINDER = 3;
  var LINE_STRIP = 4;
  var LINE_LIST = 5;
  var CUBE_LIST = 6;
  var SPHERE_LIST = 7;
  var POINTS = 8;
  var TEXT_VIEW_FACING = 9;
  var MESH_RESOURCE = 10;
  var TRIANGLE_LIST = 11;

  this.setPose(markerMsg.pose);

  var colorMaterial = makeColorMaterial(markerMsg.color.r, markerMsg.color.g, markerMsg.color.b, markerMsg.color.a);

  switch( markerMsg.type ) {
    case ARROW:
      var len = markerMsg.scale.x;
      var headLen = len * 0.23;
      var headR = markerMsg.scale.y;
      var shaftR = headR * 0.5;

      if (markerMsg.points.length == 2) {
        var p1 = pointMsgToVector3(markerMsg.points[0]);
        var p2 = pointMsgToVector3(markerMsg.points[1]);
        var dir = p1.clone().negate().addSelf(p2);
        // dir = p2 - p1;
        len = dir.length();
        headR = markerMsg.scale.y;
        shaftR = markerMsg.scale.x;

        if (markerMsg.scale.z != 0.0) {
          headLen = markerMsg.scale.z;
        }
      }

      this.add(new THREE.ArrowMarkerHelper({
        dir : dir,
        origin : p1,
        length : len,
        headLength : headLen,
        shaftDiameter : shaftR,
        headDiameter : headR,
        material : colorMaterial
      }));
      break;

    case CUBE:
      var geom = new THREE.CubeGeometry(markerMsg.scale.x, markerMsg.scale.y, markerMsg.scale.z);
      addMesh(geom, colorMaterial);
      break;

    case SPHERE:
      addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
      break;

    case CYLINDER:
      var geom = new THREE.CylinderGeometry( 0.5, 0.5, 1, 16, 1, false );
      var mesh = new THREE.Mesh(geom, colorMaterial);
      mesh.useQuaternion = true;
      mesh.quaternion.setFromAxisAngle( new THREE.Vector3(1,0,0), Math.PI*0.5 );
      mesh.scale = pointMsgToVector3( markerMsg.scale );
      this.add(mesh);
      break;

    case LINE_STRIP:
      addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
      break;

    case LINE_LIST:
      addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
      break;

    case CUBE_LIST:
      addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
      break;

    case SPHERE_LIST:
      addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
      break;

    case POINTS:
      addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
      break;

    case TEXT_VIEW_FACING:
      addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
      break;

    case MESH_RESOURCE:
      addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
      break;

    case TRIANGLE_LIST:
      var tri = new THREE.TriangleListMarkerHelper(colorMaterial, markerMsg.points, markerMsg.colors);
      tri.scale = pointMsgToVector3( markerMsg.scale );
      this.add(tri);
      break;

    default:
      addMesh(new THREE.CubeGeometry(0.1, 0.1, 0.1), colorMaterial);
      break;
  }

};

THREE.MarkerHelper.prototype = Object.create(THREE.Object3D.prototype);

THREE.MarkerHelper.prototype.setPose = function(pose) {
  this.position.x = pose.position.x;
  this.position.y = pose.position.y;
  this.position.z = pose.position.z;

  this.useQuaternion = true;
  this.quaternion = new THREE.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  this.quaternion.normalize();

  this.updateMatrixWorld();
}
