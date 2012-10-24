(function(THREE) {

  var container;
  var camera, controls, scene0, scene1, renderer;
  var pickingData = [], pickingTexture, pickingScene;
  var objects = [];
  var highlightBox;

  var directionalLight;

  var mouse = new THREE.Vector2(), offset = new THREE.Vector3(10, 10, 10);

  init();
  animate();

  var INTERSECTED, DRAGGING;

  function init() {

    // ////////////////////////////////////////////////

    container = document.getElementById("container");

    // setup camera
    camera = new THREE.PerspectiveCamera(40, window.innerWidth
        / window.innerHeight, 0.01, 1000);
    camera.position.x = 3;
    camera.position.y = 3;
    camera.position.z = 3;

    projector = new THREE.Projector();

    // setup camera mouse control
    controls = new THREE.RosOrbitControls(camera);

    // setup scene
    scene0 = new THREE.Scene();
    scene1 = new THREE.Scene();

    // scene0.eulerOrder = 'YXZ';

    // scene0.scale.x=-1;
    // scene0.scale.y=-1;

    scene0.add(directionalLight);

    // add lights
    // scene0.add(new THREE.AmbientLight(0x555555));
    directionalLight = new THREE.DirectionalLight(0xffffff);

    // attach light to camera
    scene0.add(directionalLight);

    var numCells = 50;
    var gridGeom = new THREE.PlaneGeometry(numCells, numCells, numCells,
        numCells);
    var gridMaterial = new THREE.MeshBasicMaterial({
      color : 0x999999
    });
    gridMaterial.wireframe = true;
    gridMaterial.wireframeLinewidth = 1;
    var gridObj = new THREE.Mesh(gridGeom, gridMaterial);
    scene0.add(gridObj);

    // add coordinate frame visualization
    // axes = new THREE.AxisHelper();
    // axes.scale.x=axes.scale.y=axes.scale.z=0.01;
    // scene1.add(axes);
    axes = new THREE.Axes();
    scene0.add(axes);

    imc = new THREE.InteractiveMarkerClient('ws://localhost:9090',
        '/basic_controls');
    scene0.add(imc);

    renderer = new THREE.WebGLRenderer({
      antialias : false
    });

    renderer.setClearColorHex(0x333333, 1.0);
    renderer.sortObjects = false;
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMapEnabled = false;
    renderer.autoClear = false;
    renderer.setFaceCulling(0);

    container.appendChild(renderer.domElement);
    renderer.domElement.addEventListener('mousemove', onMouseMove);
    renderer.domElement.addEventListener('mousedown', onMouseDown);
    renderer.domElement.addEventListener('mouseup', onMouseUp);

    // here you add your objects
    //THREE.Object3D._threexDomEvent.camera(camera);

    // THREEx.DomEvent = function(camera, container);
  }

  // try to call the member function fn on object obj
  // if if does not posess the function, walk up to
  // it's 'parent' object and try again, etc.
  function callFn( obj, fn )
  {
    while ( obj && obj.hasOwnProperty('parent') )
    {
      if ( obj[fn] && obj[fn] instanceof Function )
      {
        obj[fn]();
        return obj;
      } else {
        // walk up the graph
        obj = obj.parent;
      }
    }
    return undefined;
  }
  
  function onMouseDown(event) {
    callFn(INTERSECTED, 'onmousedown');
    DRAGGING = INTERSECTED;
  }

  function onMouseUp(event) {
    callFn(DRAGGING, 'onmouseup');
    if ( DRAGGING != INTERSECTED )
    {
      callFn(DRAGGING, 'onmouseout');
      callFn(INTERSECTED, 'onmouseover');
    }
    DRAGGING = undefined;
  }

  function onMouseMove(event) {
    
    event.preventDefault();

    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

    var vector = new THREE.Vector3(mouse.x, mouse.y, 0.5);
    projector.unprojectVector(vector, camera);
    var ray = new THREE.Ray(camera.position, vector.subSelf(camera.position)
        .normalize());

    var intersects = ray.intersectObject(scene0, true);
    
    var intersect;

    if (intersects.length > 0) {
      intersect = intersects[0].object;
    } else {
      container.style.cursor = 'auto';
    }
    
    if (!DRAGGING && INTERSECTED != intersect) {

      callFn(INTERSECTED, 'onmouseout');
      INTERSECTED = intersect;
      callFn(INTERSECTED, 'onmouseover');
      container.style.cursor = 'pointer';
    }

  }

  function animate() {
    requestAnimationFrame(animate);
    render();
  }

  function render() {
    controls.update();

    // put light to the top-left of the camera
    directionalLight.position = camera
        .localToWorld(new THREE.Vector3(-1, 1, 0));
    directionalLight.position.normalize();

    // clear & render regular scene
    renderer.clear(true, true, true);
    renderer.render(scene0, camera);
    // clear depth & stencil & render overlay scene
    renderer.clear(false, true, true);
    renderer.render(scene1, camera);
  }

})(THREE);
