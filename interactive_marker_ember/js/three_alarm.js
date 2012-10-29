var ThreeAlarm = Ember.Namespace.create({
  SceneView: Ember.View.extend({
    init: function() {
      this._super();

      var scene = new THREE.Scene();
      this.set('scene', scene);

      var projector = new THREE.Projector();
      this.set('projector', projector);

      var camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 1, 10000);
      camera.position.set(0, 300, 500);
      this.set('camera', camera);

      var renderer = new THREE.WebGLRenderer();
      renderer.sortObjects = false;
      renderer.setSize(500, 500);
      this.set('renderer', renderer);
    },
    didInsertElement: function() {
      console.log('did insert element')
      var renderer = this.get('renderer');
      var element = this.$()[0];
      element.appendChild(renderer.domElement);
      this.animate();
    },
    eventManager: Ember.Object.create({
      mouseMove: function(event, view) {
        var renderer = view.get('renderer');
        var element = renderer.domElement;
        var offset = $(element).offset();
        var mouse = {
          x:  ((event.clientX - offset.left) / (element.offsetWidth)) * 2 - 1,
          y: -((event.clientY - offset.top)  / (element.offsetHeight)) * 2 + 1
        };
        view.set('mouse', mouse);
      },
      mouseDown: function(event, view) {
        var pickedObject  = view.pickObject();
        var draggedObject = view.get('draggedObject');
        if (draggedObject && draggedObject !== pickedObject) {
          if (draggedObject['onmouseup']) {
            draggedObject.onmouseup(event);
          }
        }

        if (pickedObject['onmousedown']) {
          pickedObject.onmousedown(event);
        }
        view.set('draggedObject', pickedObject);
      },
      mouseUp: function(event, view) {
        var draggedObject = view.get('draggedObject');
        if (draggedObject) {
          if (draggedObject['onmouseup']) {
            draggedObject.onmouseup(event);
          }
        }
        view.set('draggedObject', null);
      }
    }),
    animate: function() {
      requestAnimationFrame(this.animate.bind(this));
      this.render();
    },
    render: function() {
      var scene    = this.get('scene');
      var camera   = this.get('camera');
      var renderer = this.get('renderer');
      renderer.render(scene, camera);
    },
    pickObject: function() {
      var intersectedObject = null;
      var mouse = this.get('mouse');
      if (mouse) {
        var projector = this.get('projector');
        var camera    = this.get('camera');
        var scene     = this.get('scene');
        var vector    = new THREE.Vector3(mouse.x, mouse.y, 1);
        projector.unprojectVector(vector, camera);

        var ray = new THREE.Ray(camera.position, vector.subSelf( camera.position ).normalize());
        var intersectedObjects = ray.intersectObjects(scene.children);

        if (intersectedObjects.length > 0) {
          intersectedObject = intersectedObjects[0].object;
        }
      }
      return intersectedObject;
    }
  })
});

