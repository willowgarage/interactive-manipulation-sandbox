define([
  'backbone'
, 'models/location'
],
function(
  Backbone
, Location
) {

  var Locations = Backbone.Collection.extend({
    model: Location

  , fetch: function() {
      this.add({
        name: 'Green room'
      , x: 0.0
      , y: 0.0
      , theta: 0.0
      });
      // this.add(greenRoom);
      // var cafeteria = new Location({
      //   name: 'cafeteria'
      // , x: 0.0
      // , y: 0.0
      // , theta: 0.0
      // });
      // this.add(cafeteria);
      // var tessasOffice = new Location({
      //   name: 'Tessa\'s Office'
      // , x: 0.0
      // , y: 0.0
      // , theta: 0.0
      // });
      // var brandonsOffice = new Location({
      //   name: 'Brandon\'s Office'
      // , x: 0.0
      // , y: 0.0
      // , theta: 0.0
      // });
      // var locations = [
      //   greenRoom
      // , cafeteria
      // , tessasOffice
      // , brandonsOffice
      // ];
      // this.add(locations);
    }

  });

  return Locations;
});



