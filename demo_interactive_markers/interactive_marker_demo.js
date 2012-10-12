(function () {

var container;
var camera, controls, scene, renderer;
var pickingData = [], pickingTexture, pickingScene;
var objects = [];
var highlightBox;

var mouse = new THREE.Vector2(), offset = new THREE.Vector3(10, 10, 10);

init();
animate();

function init() {

  //////////////////////////////////////////////////

  container = document.getElementById("container");

  // setup camera
  camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 0.01, 1000);
  camera.position.x = 2;
  camera.position.y = 2;
  camera.position.z = 2;

  // setup camera mouse control
  controls = new THREE.RosOrbitControls(camera);

  // setup scene & picking render texture
  scene = new THREE.Scene();

  //scene.eulerOrder = 'YXZ';

  //scene.scale.x=-1;
  //scene.scale.y=-1;
  
  scene.add( directionalLight );

  // add lights
  scene.add(new THREE.AmbientLight(0x555555));

  var directionalLight = new THREE.PointLight( 0xffffff );
  directionalLight.position.x = 50;
  directionalLight.position.y = 50;
  directionalLight.position.z = 50;
  directionalLight.position.normalize();
  scene.add( directionalLight );

  var directionalLight = new THREE.PointLight( 0xffffff );
  directionalLight.position.x = 50;
  directionalLight.position.y = -50;
  directionalLight.position.z = -50;
  directionalLight.position.normalize();
  scene.add( directionalLight );

  var gridGeom = new THREE.PlaneGeometry( 10, 10, 10, 10 );
  var gridMaterial = new THREE.MeshBasicMaterial({ color: 0x999999 });
  gridMaterial.wireframe=true;
  gridMaterial.wireframeLinewidth=1.5;
  var gridObj = new THREE.Mesh( gridGeom, gridMaterial );
  scene.add( gridObj );
  
  // add coordinate frame visualization
  //axes = new THREE.AxisHelper();
  //axes.scale.x=axes.scale.y=axes.scale.z=0.01;
  axes = new THREE.Axes();
  scene.add(axes);

  imc = new THREE.InteractiveMarkerClient( 'ws://localhost:9090', '/basic_controls' );
  scene.add(imc);

  renderer = new THREE.WebGLRenderer({
    antialias : true
  });

  renderer.setClearColorHex( 0x333333, 1.0 );  
  renderer.sortObjects = true;
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMapEnabled = false;

  container.appendChild(renderer.domElement);

  renderer.domElement.addEventListener('mousemove', onMouseMove);
}

//

function onMouseMove(e) {
  mouse.x = e.clientX;
  mouse.y = e.clientY;
}

function animate() {
  requestAnimationFrame(animate);
  render();
}

function render() {
  controls.update();
  renderer.render(scene, camera);
}

})();
