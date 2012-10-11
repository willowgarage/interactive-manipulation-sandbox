/**
 * @author dgossow@willowgarage.com
 */
THREE.Widget3D = function () {

THREE.Object3D()
    THREE.Object3D.call( this );
    THREE.Object3D();

};

THREE.Widget3D.prototype = new THREE.Object3D();

THREE.Widget3D.prototype.onMouseOver = function () {
    // ...
}

THREE.Widget3D.prototype.constructor = THREE.Axes;

var widget = new THREE.Widget3D()
