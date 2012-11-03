(function(THREE) {

  var container;
  var camera, controls, scene0, scene1, renderer;
  var pickingData = [], pickingTexture, pickingScene;
  var objects = [];
  var highlightBox;

  var directionalLight;

  var mouseHandler;

  var mouse = new THREE.Vector2(), offset = new THREE.Vector3(10, 10, 10);

  init();
  animate();

  var INTERSECTED, INTERSECTION, DRAGGING;
  
  var imc,imm;

  function init() {

    // ////////////////////////////////////////////////

    container = document.getElementById("container");

    // setup camera
    camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.01, 1000);
    camera.position.x = 3;
    camera.position.y = 3;
    camera.position.z = 3;

    projector = new THREE.Projector();

    // setup scene
    scene0 = new THREE.Scene();
    scene1 = new THREE.Scene();

    // setup camera mouse control
    controls = new THREE.RosOrbitControls(camera);

    scene0.add(directionalLight);

    // add lights
    // scene0.add(new THREE.AmbientLight(0x555555));
    directionalLight = new THREE.DirectionalLight(0xffffff);

    // attach light to camera
    scene0.add(directionalLight);

    var numCells = 50;
    var gridGeom = new THREE.PlaneGeometry(numCells, numCells, numCells, numCells);
    var gridMaterial = new THREE.MeshBasicMaterial({
      color : 0x999999
    });
    gridMaterial.wireframe = true;
    gridMaterial.wireframeLinewidth = 1;
    gridMaterial.transparent = true;
    var gridObj = new THREE.Mesh(gridGeom, gridMaterial);
    scene1.add(gridObj);

    // add coordinate frame visualization
    // axes = new THREE.AxisHelper();
    // axes.scale.x=axes.scale.y=axes.scale.z=0.01;
    // scene1.add(axes);
    axes = new THREE.Axes();
    scene1.add(axes);

    var ros = new ROS('ws://localhost:9090');

    imc = new interactive_markers.Client(ros);
    imm = new THREE.InteractiveMarkerManager(scene0, imc);
    
    imc.subscribe('/basic_controls');

    renderer = new THREE.WebGLRenderer({
      antialias : true
    });

    renderer.setClearColorHex(0x333333, 1.0);
    renderer.sortObjects = false;
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMapEnabled = false;
    renderer.autoClear = false;
    renderer.setFaceCulling(0);

    container.appendChild(renderer.domElement);
    //renderer.domElement.addEventListener('mousemove', onMouseMove, false);
    //renderer.domElement.addEventListener('mousedown', onMouseDown, false);
    //renderer.domElement.addEventListener('mouseup', onMouseUp, false);
    //renderer.domElement.addEventListener('mousewheel', onMouseWheel, false);

    renderer.domElement.addEventListener('contextmenu', function(event) {
      event.preventDefault();
    }, false);

    mouseHandler = new THREE.MouseHandler(renderer, camera, scene0);
  }

  // try to call the member function fn on object obj
  // if if does not posess the function, walk up to
  // it's 'parent' object and try again, etc.
  function callFn(obj, fn, event) {

    // make the event cancelable
    event.cancelBubble = false;
    event.stopPropagation = function() {
      event.cancelBubble = true;
    }
    while (obj) {
      if (obj[fn] && obj[fn] instanceof Function) {
        obj[fn](event);
        if (event.cancelBubble) {
          return;
        }
      }

      // walk up the graph
      if (obj.hasOwnProperty('parent')) {
        obj = obj.parent;
      } else {
        return;
      }
    }

    // if we arrive here, no object has stopped propagation
    if (controls[fn] && controls[fn] instanceof Function) {
      controls[fn](event);
      return;
    }
  }

  function onMouseWheel(event) {
    event.intersectionPoint = INTERSECTION;
    callFn(INTERSECTED, 'onmousewheel', event);
    DRAGGING = INTERSECTED;
  }

  function onMouseDown(event) {
    event.intersectionPoint = INTERSECTION;
    callFn(INTERSECTED, 'onmousedown', event);
    DRAGGING = INTERSECTED;
  }

  function onMouseUp(event) {
    event.intersectionPoint = INTERSECTION;
    callFn(DRAGGING, 'onmouseup', event);
    if (DRAGGING !== INTERSECTED) {
      callFn(DRAGGING, 'onmouseout', event);
      callFn(INTERSECTED, 'onmouseover', event);
    }
    DRAGGING = undefined;
  }

  function onMouseMove(event) {

    // keep this event from interacting with the DOM
    event.preventDefault();

    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

    // trace a ray through the mouse cursor position
    var vector = new THREE.Vector3(mouse.x, mouse.y, 0.5);
    projector.unprojectVector(vector, camera);
    var ray = new THREE.Ray(camera.position, vector.subSelf(camera.position).normalize());

    event.mouseRay = ray;
    event.mousePos = mouse;
    event.camera = camera;

    // if we're not dragging, see if there is something new under the mouse cursor
    var intersectedObjs = ray.intersectObject(scene0, true);
    console.log(intersectedObjs);
    var intersectedObj;

    if (intersectedObjs.length > 0) {
      intersectedObj = intersectedObjs[0].object;
      INTERSECTION = intersectedObjs[0].point;
    } else {
      container.style.cursor = 'auto';
      intersectedObj = controls;
      INTERSECTION = undefined;
    }

    // store intersection point in event
    event.intersectionPoint = INTERSECTION;

    if (DRAGGING === undefined && INTERSECTED !== intersectedObj) {
      callFn(INTERSECTED, 'onmouseout', event);
      INTERSECTED = intersectedObj;
      callFn(INTERSECTED, 'onmouseover', event);
      container.style.cursor = 'pointer';
    } else if (DRAGGING) {
      callFn(DRAGGING, 'onmousemove', event);
    }

  }

  function animate() {
    requestAnimationFrame(animate);
    render();
  }

  function render() {
    controls.update();

    // put light to the top-left of the camera
    directionalLight.position = camera.localToWorld(new THREE.Vector3(-1, 1, 0));
    directionalLight.position.normalize();

    // clear & render regular scene
    renderer.clear(true, true, true);
    renderer.render(scene0, camera);
    // clear depth & stencil & render overlay scene
    //renderer.clear(false, true, true);
    renderer.render(scene1, camera);
  }

})(THREE);
