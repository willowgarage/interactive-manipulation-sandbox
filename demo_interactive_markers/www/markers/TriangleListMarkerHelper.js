/**
 * @author dgossow http://github.com/dgossow
 * @author WestLangley / http://github.com/WestLangley
 * @author zz85 / https://github.com/zz85
 */

THREE.TriangleListMarkerHelper = function ( material, vertices )
{
  THREE.Object3D.call( this );
  
  if ( material === undefined ) material = new THREE.MeshBasicMaterial();
  
  material.side = THREE.DoubleSide;
  
  var geometry = new THREE.Geometry();
  
  for ( i = 0; i < vertices.length; i++ ) 
  {
    geometry.vertices.push( new THREE.Vector3( vertices[i].x, vertices[i].y, vertices[i].z ) );
  }
  for ( i = 0; i < vertices.length; i+=3 )
  {
    geometry.faces.push( new THREE.Face3( i, i+1, i+2 ) );
    // duplicate face backwards to get around the back face culling
    // (mesh.doubleSided seems not to have any effect.) 
  }
  
  this.mesh = new THREE.Mesh( geometry, material );
  this.add( this.mesh );
};

THREE.TriangleListMarkerHelper.prototype = Object.create( THREE.Object3D.prototype );

THREE.TriangleListMarkerHelper.prototype.setColor = function ( hex ) 
{
  this.mesh.material.color.setHex( hex );
};
