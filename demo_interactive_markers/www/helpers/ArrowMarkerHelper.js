/**
 * @author dgossow
 * @author WestLangley / http://github.com/WestLangley
 * @author zz85 / https://github.com/zz85
 *
 * Creates an arrow for visualizing directions
 *
 * Parameters:
 *  dir - Vector3
 *  origin - Vector3
 *  length - Number
 *  hex - color in hex value
 */

THREE.ArrowMarkerHelper = function ( dir, origin, length, headLength, shaftRadius, headRadius, material ) {

        THREE.Object3D.call( this );

        if ( material === undefined ) material = new THREE.MeshBasicMaterial();
        if ( length === undefined ) length = 20;

        var shaftLength = length-headLength;

        var shaftGeometry = new THREE.CylinderGeometry( shaftRadius, shaftRadius, shaftLength, 12, 1);

        this.shaft = new THREE.Mesh( shaftGeometry, material );
        this.shaft.position.set( 0, shaftLength*0.5, 0 );
        this.add( this.shaft );

        //function ( radiusTop, radiusBottom, height, segmentsRadius, segmentsHeight, openEnded ) {
        var coneGeometry = new THREE.CylinderGeometry( 0, headRadius, headLength, 12, 1 );

        this.cone = new THREE.Mesh( coneGeometry, material );
        this.cone.position.set( 0, shaftLength+(headLength * 0.5), 0 );
        this.add( this.cone );

        if ( origin instanceof THREE.Vector3 ) this.position = origin;

        this.setDirection( dir );
};

THREE.ArrowMarkerHelper.prototype = Object.create( THREE.Object3D.prototype );

THREE.ArrowMarkerHelper.prototype.setDirection = function ( dir ) {

        var axis = new THREE.Vector3( 0, 1, 0 ).crossSelf( dir );

        var radians = Math.acos( new THREE.Vector3( 0, 1, 0 ).dot( dir.clone().normalize() ) );

        this.matrix = new THREE.Matrix4().makeRotationAxis( axis.normalize(), radians );

        this.rotation.setEulerFromRotationMatrix( this.matrix, this.eulerOrder );

};

THREE.ArrowMarkerHelper.prototype.setLength = function ( length ) {

        this.scale.set( length, length, length );

};

THREE.ArrowMarkerHelper.prototype.setColor = function ( hex ) {

        this.line.material.color.setHex( hex );
        this.cone.material.color.setHex( hex );

};
