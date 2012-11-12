/**
 * @author dgossow http://github.com/dgossow
 * @author WestLangley / http://github.com/WestLangley
 * @author zz85 / https://github.com/zz85
 * 
 * Creates an arrow for visualizing directions
 * 
 * Parameters: dir - Vector3 origin - Vector3 length - Number hex - color in hex
 * value
 */

THREE.ArrowMarkerHelper = function( options ) {

  var origin = options.origin || new THREE.Vector3(0,0,0);
  var dir = options.dir || new THREE.Vector3(1,0,0);
  
  var length = options.length || 1;
  var headLength = options.headLength || 0.2;
  
  var shaftDiameter = options.shaftDiameter || 0.05;
  var headDiameter = options.headDiameter || 0.1;
  
  var material = options.material || new THREE.MeshBasicMaterial();

  var shaftLength = length - headLength;

  // create and merge geometry
  var geometry = new THREE.CylinderGeometry(shaftDiameter * 0.5,
      shaftDiameter * 0.5, shaftLength, 12, 1);

  var m = new THREE.Matrix4;
  m.setPosition( new THREE.Vector3(0, shaftLength * 0.5, 0) );
  geometry.applyMatrix(m);
  
  var coneGeometry = new THREE.CylinderGeometry(0, headDiameter * 0.5,
      headLength, 12, 1);

  m.setPosition( new THREE.Vector3(0, shaftLength + (headLength * 0.5), 0) );
  coneGeometry.applyMatrix(m);
      
  THREE.GeometryUtils.merge( geometry, coneGeometry );

  THREE.Mesh.call(this,geometry,material);

  this.position = origin;
  this.setDirection(dir);
};

THREE.ArrowMarkerHelper.prototype = Object.create(THREE.Mesh.prototype);

THREE.ArrowMarkerHelper.prototype.setDirection = function(dir) {

  var axis = new THREE.Vector3(0, 1, 0).crossSelf(dir);

  var radians = Math.acos(new THREE.Vector3(0, 1, 0).dot(dir.clone()
      .normalize()));

  this.matrix = new THREE.Matrix4().makeRotationAxis(axis.normalize(), radians);

  this.rotation.setEulerFromRotationMatrix(this.matrix, this.eulerOrder);

};

THREE.ArrowMarkerHelper.prototype.setLength = function(length) {

  this.scale.set(length, length, length);

};

THREE.ArrowMarkerHelper.prototype.setColor = function(hex) {

  this.line.material.color.setHex(hex);
  this.cone.material.color.setHex(hex);
};
