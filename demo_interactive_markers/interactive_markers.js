var container;
var camera, controls, scene, renderer;
var pickingData = [], pickingTexture, pickingScene;
var objects = [];
var highlightBox;

var mouse = new THREE.Vector2(),
	offset = new THREE.Vector3(10, 10, 10);

init();
animate();

function applyVertexColors(g, c) 
{
	g.faces.forEach(function(f){
		var n = (f instanceof THREE.Face3) ? 3 : 4;
		for(var j=0; j<n; j++){
			f.vertexColors[j] = c;
		}
	});
}

function init() {

	//////////////////////////////////////////////////

	container = document.getElementById("container");
	
	// setup camera
	camera = new THREE.PerspectiveCamera( 70, window.innerWidth / window.innerHeight, 0.01, 1000 );
	camera.position.z = 2;

	// setup camera mouse control
	controls = new THREE.OrbitControls( camera );
	controls.center.x = 0;

	// setup scene & picking render texture
	scene = new THREE.Scene();
	
	// add lights
	scene.add( new THREE.AmbientLight( 0x555555 ) );
	
	// add coordinate frame visualization
	axes = new THREE.Axes();
    scene.add( axes );

    console.log(this);
    
	renderer = new THREE.WebGLRenderer( { antialias: true, clearColor: 0x333333 } );
	renderer.sortObjects = true;
	renderer.setSize( window.innerWidth, window.innerHeight );
	renderer.shadowMapEnabled = false;

	container.appendChild( renderer.domElement );
	
	renderer.domElement.addEventListener('mousemove', onMouseMove);
}
//

function onMouseMove(e) {
	mouse.x = e.clientX;
	mouse.y = e.clientY;
}

function animate() 
{
	requestAnimationFrame( animate );
	render();
}

function render() 
{
	controls.update();
	renderer.render( scene, camera );
}

