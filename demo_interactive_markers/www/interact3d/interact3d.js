var INTERACT3D = INTERACT3D || {

  projector : new THREE.Projector(),

  findClosestPoint : function(target_ray, mouse_ray) {
    // Find the closest point on target_ray to any point on mouse_ray.
    // Math taken from http://paulbourke.net/geometry/lineline3d/
    // line P1->P2 is target_ray
    // line P3->P4 is mouse_ray

    var v13 = new THREE.Vector3;
    v13.sub(target_ray.origin, mouse_ray.origin);
    var v43 = mouse_ray.direction.clone();
    var v21 = target_ray.direction.clone();
    var d1343 = v13.dot(v43);
    var d4321 = v43.dot(v21);
    var d1321 = v13.dot(v21);
    var d4343 = v43.dot(v43);
    var d2121 = v21.dot(v21);

    var denom = d2121 * d4343 - d4321 * d4321;
    if (Math.abs(denom) <= 0.0001) {
      return undefined;
    }
    var numer = d1343 * d4321 - d1321 * d4343;

    var mua = numer / denom;
    return mua;
  },

  closestAxisPoint : function(axisRay, camera, mousePos) {
    // project axis onto screen
    var o = axisRay.origin.clone();
    projector.projectVector(o, camera);

    var o2 = axisRay.direction.clone().addSelf(axisRay.origin);
    projector.projectVector(o2, camera);

    // d is the axis vector in screen space
    var d = o2.clone().subSelf(o);
    // d = o2-o;

    // t is the 2d ray param of perpendicular projection
    // of mousePos onto o
    var tmp = new THREE.Vector2;
    var t = tmp.sub(mousePos, o).dot(d) / d.dot(d);
    // t = (mousePos - o) * d / (d*d);

    // mp is the final 2d-projected mouse pos
    var mp = new THREE.Vector2;
    mp.add(o, d.clone().multiplyScalar(t));
    // mp = o + d*t;

    // go back to 3d by shooting a ray
    var vector = new THREE.Vector3(mp.x, mp.y, 0.5);
    projector.unprojectVector(vector, camera);
    var mpRay = new THREE.Ray(camera.position, vector.subSelf(camera.position).normalize());
    var mua = INTERACT3D.findClosestPoint(axisRay, mpRay, mua);

    return mua;
  },

  getWebglObjects : function(scene, objects, renderList) {
    var objlist = scene.__webglObjects;
    // get corresponding webgl objects
    for (var c = 0; c < objects.length; c++) {
      if ( !objects[c] ) {
        continue;
      }
      for (var o = objlist.length - 1; o >= 0; o--) {
        if (objlist[o].object === objects[c]) {
          renderList.push(objlist[o]);
          break;
        }
      }
      // recurse into children
      this.getWebglObjects(scene, objects[c].children, renderList);
    }
  },

  renderHighlight : function(renderer, scene, camera, objects) {

    if (!objects || objects.length === 0) {
      return;
    }

    // get webgl objects
    var renderList = [];
    this.getWebglObjects(scene, objects, renderList);

    //define highlight material
    var overrideMaterial = new THREE.MeshBasicMaterial({
      fog : false,
      opacity : 0.5,
      depthTest : true,
      depthWrite : false,
      polygonOffset : true,
      polygonOffsetUnits : -1
    });
    scene.overrideMaterial = overrideMaterial;

    // swap render lists, render, undo
    var oldWebglObjects = scene.__webglObjects;
    scene.__webglObjects = renderList;

    renderer.render(scene, camera);

    scene.__webglObjects = oldWebglObjects;
    scene.overrideMaterial = null;
  }
}
