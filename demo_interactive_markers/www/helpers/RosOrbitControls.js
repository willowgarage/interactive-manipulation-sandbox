/**
 * @author dgossow / https://github.com/dgossow
 * @author qiao / https://github.com/qiao
 * @author mrdoob / http://mrdoob.com
 * @author alteredq / http://alteredqualia.com/
 */

// behaves like THREE.OrbitControls, but uses
// right-handed coords and z as up vector

THREE.RosOrbitControls = function ( object, domElement ) {

	THREE.EventTarget.call( this );

	this.object = object;
	
	// hack the tracked object (camera) to be right-handed with z=up
  this.object.scale.x=-1;
  this.object.scale.y=-1;
  this.object.up = new THREE.Vector3( 0, 0, -1 );
  
	this.domElement = ( domElement !== undefined ) ? domElement : document;

	// API

	this.center = new THREE.Vector3();

	this.userZoom = true;
	this.userZoomSpeed = 1.0;

	this.userRotate = true;
	this.userRotateSpeed = 1.0;

	this.autoRotate = false;
	this.autoRotateSpeed = 2.0; // 30 seconds per round when fps is 60

	// internals

	var scope = this;

	var EPS = 0.000001;
	var PIXELS_PER_ROUND = 1800;

	var rotateStart = new THREE.Vector2();
	var rotateEnd = new THREE.Vector2();
	var rotateDelta = new THREE.Vector2();

	var zoomStart = new THREE.Vector2();
	var zoomEnd = new THREE.Vector2();
	var zoomDelta = new THREE.Vector2();

	var phiDelta = 0;
	var thetaDelta = 0;
	var scale = 1;

	var lastPosition = new THREE.Vector3();

	var STATE = { NONE : -1, ROTATE : 0, ZOOM : 1 };
	var state = STATE.NONE;

	// events

	var changeEvent = { type: 'change' };


	this.rotateLeft = function ( angle ) {

		if ( angle === undefined ) {

			angle = getAutoRotationAngle();

		}

		thetaDelta -= angle;

	};

	this.rotateRight = function ( angle ) {

		if ( angle === undefined ) {

			angle = getAutoRotationAngle();

		}

		thetaDelta += angle;

	};

	this.rotateUp = function ( angle ) {

		if ( angle === undefined ) {

			angle = getAutoRotationAngle();

		}

		phiDelta -= angle;

	};

	this.rotateDown = function ( angle ) {

		if ( angle === undefined ) {

			angle = getAutoRotationAngle();

		}

		phiDelta += angle;

	};

	this.zoomIn = function ( zoomScale ) {

		if ( zoomScale === undefined ) {

			zoomScale = getZoomScale();

		}

		scale /= zoomScale;

	};

	this.zoomOut = function ( zoomScale ) {

		if ( zoomScale === undefined ) {

			zoomScale = getZoomScale();

		}

		scale *= zoomScale;

	};

	this.update = function () {

		var position = this.object.position;
		var offset = position.clone().subSelf( this.center )

    // x->y, y->z, z->x

		// angle from z-axis around y-axis
		var theta = Math.atan2( offset.y, offset.x );

		// angle from y-axis

		var phi = Math.atan2( Math.sqrt( offset.y * offset.y + offset.x * offset.x ), offset.z );

		if ( this.autoRotate ) {

			this.rotateLeft( getAutoRotationAngle() );

		}

		theta += thetaDelta;
		phi += phiDelta;

		// restrict phi to be betwee EPS and PI-EPS

		phi = Math.max( EPS, Math.min( Math.PI - EPS, phi ) );

		var radius = offset.length();
		offset.y = radius * Math.sin( phi ) * Math.sin( theta );
		offset.z = radius * Math.cos( phi );
		offset.x = radius * Math.sin( phi ) * Math.cos( theta );
		offset.multiplyScalar( scale );

		position.copy( this.center ).addSelf( offset );

		this.object.lookAt( this.center );

		thetaDelta = 0;
		phiDelta = 0;
		scale = 1;

		if ( lastPosition.distanceTo( this.object.position ) > 0 ) {

			this.dispatchEvent( changeEvent );

			lastPosition.copy( this.object.position );

		}
	};


	function getAutoRotationAngle() {

		return 2 * Math.PI / 60 / 60 * scope.autoRotateSpeed;

	}

	function getZoomScale() {

		return Math.pow( 0.95, scope.userZoomSpeed );

	}

	function onMouseDown( event3d ) {
	  
	  var event = event3d.domEvent;

		event.preventDefault();

		if ( event.button === 0 || event.button === 2 ) {

			state = STATE.ROTATE;

			rotateStart.set( event.clientX, event.clientY );

		} else if ( event.button === 1 ) {

			state = STATE.ZOOM;

			zoomStart.set( event.clientX, event.clientY );

		}

	}

	function onMouseMove( event3d ) {

		    var event = event3d.domEvent;


		if ( state === STATE.ROTATE ) {

			rotateEnd.set( event.clientX, event.clientY );
			rotateDelta.sub( rotateEnd, rotateStart );

			scope.rotateLeft( 2 * Math.PI * rotateDelta.x / PIXELS_PER_ROUND * scope.userRotateSpeed );
			scope.rotateUp( 2 * Math.PI * rotateDelta.y / PIXELS_PER_ROUND * scope.userRotateSpeed );

			rotateStart.copy( rotateEnd );

		} else if ( state === STATE.ZOOM ) {

			zoomEnd.set( event.clientX, event.clientY );
			zoomDelta.sub( zoomEnd, zoomStart );

			if ( zoomDelta.y > 0 ) {

				scope.zoomIn();

			} else {

				scope.zoomOut();

			}

			zoomStart.copy( zoomEnd );

		}

	}

	function onMouseUp( event3d ) {

    var event = event3d.domEvent;

		if ( ! scope.userRotate ) return;

		state = STATE.NONE;

	}

	function onMouseWheel( event3d ) {

		if ( ! scope.userZoom ) return;

    var event = event3d.domEvent;

		if ( event.wheelDelta > 0 ) {

			scope.zoomOut();

		} else {

			scope.zoomIn();

		}

	}
	
	THREE.EventTarget.call(this);
  this.addEventListener('mousedown',onMouseDown);
  this.addEventListener('mouseup',onMouseUp);
  this.addEventListener('mousemove',onMouseMove);
  this.addEventListener('mousewheel',onMouseWheel);
  //this.onmousedown = function(event){ onMouseDown(event); }
  //this.onmouseup = function(event){ onMouseUp(event); }
  //this.onmousemove = function(event){ onMouseMove(event); }
  //this.onmousewheel = function(event){ onMouseWheel(event); }
};
