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

  THREE.Object3D.call(this);

  var origin = options.origin || new THREE.Vector3(0,0,0);
  var dir = options.dir || new THREE.Vector3(1,0,0);
  
  var length = options.length || 1;
  var headLength = options.headLength || 0.2;
  
  var shaftDiameter = options.shaftDiameter || 0.05;
  var headDiameter = options.headDiameter || 0.1;
  
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
  
  var shaftLength = length - headLength;

  var shaftGeometry = new THREE.CylinderGeometry(shaftDiameter * 0.5,
      shaftDiameter * 0.5, shaftLength, 12, 1);

  this.shaft = new THREE.Mesh(shaftGeometry, this.material);
  this.shaft.position.set(0, shaftLength * 0.5, 0);
  this.add(this.shaft);

  var coneGeometry = new THREE.CylinderGeometry(0, headDiameter * 0.5,
      headLength, 12, 1);

  this.cone = new THREE.Mesh(coneGeometry, this.material);
  this.cone.position.set(0, shaftLength + (headLength * 0.5), 0);
  this.add(this.cone);

  this.position = origin;
  this.setDirection(dir);
  
  this.dragging = false;

  //this.scale = new THREE.Vector3( 1, 1, 1 );
  
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
};

THREE.ArrowMarkerHelper.prototype = Object.create(THREE.Object3D.prototype);

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
