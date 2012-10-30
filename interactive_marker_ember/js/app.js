App = Ember.Application.create();

App.ApplicationController = Ember.Controller.extend();
App.ApplicationView = Ember.View.extend({
  templateName: 'application'
});

App.MySceneController = Ember.ArrayController.extend();
App.MySceneView = ThreeAlarm.SceneView.extend({
  templateName: 'my-scene',
  init: function() {
    this._super();
    var scene = this.get('scene');

    var camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 1, 10000);
    camera.position.set(0, 300, 500);
    this.set('camera', camera);

    var light = new THREE.DirectionalLight( 0xffffff, 2 );
    light.position.set( 1, 1, 1 ).normalize();
    scene.add(light);

    var light = new THREE.DirectionalLight( 0xffffff );
    light.position.set( -1, -1, -1 ).normalize();
    scene.add(light);

    var geometry = new THREE.CubeGeometry(20, 20, 20);
    for (var i = 0; i < 2000; i ++) {
      var object = new THREE.Mesh(geometry, new THREE.MeshLambertMaterial( { color: Math.random() * 0xffffff } ));
      object.onmousedown = function(event) {
        this.material.emissive.setHex(0x00ff00);
      };
      object.onmouseup = function(event) {
        this.material.emissive.setHex(0xff0000);
      };
      object.position.x = Math.random() * 800 - 400;
      object.position.y = Math.random() * 800 - 400;
      object.position.z = Math.random() * 800 - 400;

      object.rotation.x = ( Math.random() * 360 ) * Math.PI / 180;
      object.rotation.y = ( Math.random() * 360 ) * Math.PI / 180;
      object.rotation.z = ( Math.random() * 360 ) * Math.PI / 180;

      object.scale.x = Math.random() + 0.5;
      object.scale.y = Math.random() + 0.5;
      object.scale.z = Math.random() + 0.5;

      scene.add(object);
    }
  }
});

App.Router = Ember.Router.extend({
  root: Ember.Route.extend({
    interactiveMarkers: Ember.Route.extend({
      route: '/',
      connectOutlets: function(router) {
        router.get('applicationController').connectOutlet('myScene', []);
      }
    })
  })
});

App.initialize();

