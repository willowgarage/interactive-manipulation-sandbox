define([
  'ember',
  'app',
  'three',
  'ROS',
  'libs/interactivemarkersjs/improxy',
  'libs/interactivemarkersjs/imthree',
  'libs/interactivemarkersjs/markersthree',
  'libs/interactivemarkersjs/threeinteraction',
  'libs/interactivemarkersjs/examples/include/helpers/RosAxisHelper',
  'libs/interactivemarkersjs/examples/include/helpers/RosOrbitControls',
  'text!templates/markers.handlebars'
], function(Ember, App, THREE, ROS, ImProxy, ImThree, MarkersThree, ThreeInteraction, RosAxes, RosOrbit, markersHtml) {

  var camera, cameraControls, scene, renderer;

  var selectableObjs;

  var directionalLight;

  var mouseHandler, highlighter;

  var imClient, imViewer;

  function init(ros) {

    var w=640;
    var h=480;
    // setup camera
    camera = new THREE.PerspectiveCamera(40, w/h, 0.01, 1000);
    camera.position.x = 3;
    camera.position.y = 3;
    camera.position.z = 3;

    // setup scene
    scene = new THREE.Scene();

    // setup camera mouse control
    cameraControls = new THREE.RosOrbitControls(camera);

    // add node to host selectable objects
    selectableObjs = new THREE.Object3D;
    scene.add(selectableObjs);

    // add lights
    scene.add(new THREE.AmbientLight(0x555555));
    directionalLight = new THREE.DirectionalLight(0xffffff);
    scene.add(directionalLight);

    // add x/y grid
    var numCells = 50;
    var gridGeom = new THREE.PlaneGeometry(numCells, numCells, numCells, numCells);
    var gridMaterial = new THREE.MeshBasicMaterial({
      color : 0x999999,
      wireframe : true,
      wireframeLinewidth : 1,
      transparent : true
    });
    var gridObj = new THREE.Mesh(gridGeom, gridMaterial);
    scene.add(gridObj);

    // add coordinate frame visualization
    axes = new THREE.Axes();
    scene.add(axes);

    renderer = new THREE.WebGLRenderer({
      antialias : true
    });
    renderer.setClearColorHex(0x333333, 1.0);
    renderer.sortObjects = false;
    renderer.setSize(w,h);
    renderer.shadowMapEnabled = false;
    renderer.autoClear = false;

    var container = document.getElementById("container");
    container.appendChild(renderer.domElement);

    // propagates mouse events to three.js objects
    mouseHandler = new ThreeInteraction.MouseHandler(renderer, camera, selectableObjs, cameraControls);

    // highlights the receiver of mouse events
    highlighter = new ThreeInteraction.Highlighter(mouseHandler);

    var meshBaseUrl = 'http://blh:8000/resources/';

    // show interactive markers
    imClient = new ImProxy.Client(ros);
    imViewer = new ImThree.Viewer(selectableObjs, camera, imClient, meshBaseUrl);

    subscribe('/pr2_marker_control');
  }

  subscribe = function( topic ) {
    imClient.subscribe(topic);
  }

  unsubscribe = function( topic ) {
    imClient.unsubscribe();
  }

  function animate() {

    cameraControls.update();

    // put light to the top-left of the camera
    directionalLight.position = camera.localToWorld(new THREE.Vector3(-1, 1, 0));
    directionalLight.position.normalize();

    renderer.clear(true, true, true);
    renderer.render(scene, camera);

    highlighter.renderHighlight(renderer, scene, camera);

    requestAnimationFrame(animate);
  }

  App.MarkersView = Ember.View.extend({
    template: Ember.Handlebars.compile(markersHtml),
    didInsertElement : function() {
      var content = this.get('controller').get('content');
      init(content.ros);
      animate();
    }
  });

});
