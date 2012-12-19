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

  App.MarkersView = Ember.View.extend({
    template: Ember.Handlebars.compile(markersHtml),
    didInsertElement : function() {
      this.start();
      this.animate();
    },
    start : function() {
      var width  = 640;
      var height = 480;

      // Setup camera
      var camera = new THREE.PerspectiveCamera(40, width/height, 0.01, 1000);
      camera.position.x = 3;
      camera.position.y = 3;
      camera.position.z = 3;
      this.set('camera', camera);

      // Setup scene
      var scene = new THREE.Scene();
      this.set('scene', scene);

      // Setup camera mouse control
      var cameraControls = new THREE.RosOrbitControls(camera);
      this.set('cameraControls', cameraControls);

      // Add node to host selectable objects
      var selectableObjects = new THREE.Object3D();
      scene.add(selectableObjects);
      this.set('selectableObjects', selectableObjects);

      // Add lights
      scene.add(new THREE.AmbientLight(0x555555));
      var directionalLight = new THREE.DirectionalLight(0xffffff);
      this.set('directionalLight', directionalLight);
      scene.add(directionalLight);

      // Add x/y grid
      var cellCount = 50;
      var gridGeometry = new THREE.PlaneGeometry(cellCount, cellCount, cellCount, cellCount);
      var gridMaterial = new THREE.MeshBasicMaterial({
        color : 0x999999,
        wireframe : true,
        wireframeLinewidth : 1,
        transparent : true
      });
      var gridMesh = new THREE.Mesh(gridGeometry, gridMaterial);
      scene.add(gridMesh);

      // Add coordinate frame visualization
      var axes = new THREE.Axes();
      scene.add(axes);

      var renderer = new THREE.WebGLRenderer({
        antialias : true
      });
      renderer.setClearColorHex(0x333333, 1.0);
      renderer.sortObjects = false;
      renderer.setSize(width, height);
      renderer.shadowMapEnabled = false;
      renderer.autoClear = false;
      this.set('renderer', renderer);

      var container = document.getElementById('container');
      container.appendChild(renderer.domElement);

      // Propagates mouse events to three.js objects
      var mouseHandler = new ThreeInteraction.MouseHandler(renderer, camera, selectableObjects, cameraControls);

      // Highlights the receiver of mouse events
      var highlighter = new ThreeInteraction.Highlighter(mouseHandler);
      this.set('highlighter', highlighter);

      // Only subscribe to interactive markers once the robot model has been loaded
      var robot = this.get('controller').get('content');
      // TL: This is a hack to make the markers tab work. If we get to this tab
      // and the robot has already been loaded, then status_code will be 1 and
      // the observer will never fire. However if we get to this tab before the
      // robot has loaded, then robot.ros has not been created yet. So we need
      // a better way to only call subscribe when there is a ROS connection
      // open.
      if (robot.get('status_code') == 1) {
        this.subscribeToMarkers();
      } else {
        robot.addObserver('status_code', this, 'subscribeToMarkers');
      }
    },

    willDestroyElement: function() {
      // Remove the subscription when we switch away from this tab
      var robot = this.get('controller').get('content');
      robot.removeObserver('status_code', this, 'subscribeToMarkers');
    },

    subscribeToMarkers: function() {
      var robot = this.get('controller').get('content');
      var status_code = robot.get('status_code');
      if (status_code == 1) {
        // Show interactive markers
        var content = this.get('controller').get('content');
        var imClient = new ImProxy.Client(content.ros);
        // TODO: this should most definitely not be hardcoded to blh
        var meshBaseUrl = 'http://blh.willowgarage.com:8000/resources/';
        var imViewer = new ImThree.Viewer(this.get('selectableObjects'), this.get('camera'), imClient, meshBaseUrl);

        imClient.subscribe('/pr2_marker_control');
      }
    },

    animate : function() {
      var camera = this.get('camera');
      var cameraControls = this.get('cameraControls');
      cameraControls.update();

      // Put light to the top-left of the camera
      var directionalLight = this.get('directionalLight');
      directionalLight.position = camera.localToWorld(new THREE.Vector3(-1, 1, 0));
      directionalLight.position.normalize();

      var renderer = this.get('renderer');
      var scene = this.get('scene');
      renderer.clear(true, true, true);
      renderer.render(scene, camera);

      var highlighter = this.get('highlighter');
      highlighter.renderHighlight(renderer, scene, camera);

      requestAnimationFrame(this.animate.bind(this));
    }
  });
});

