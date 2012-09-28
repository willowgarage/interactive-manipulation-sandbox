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
      var greenRoom = new Location({
        name: 'Green room'
      , x: 0.0
      , y: 0.0
      , theta: 0.0
      });
      var cafeteria = new Location({
        name: 'Cafeteria'
      , x: 0.0
      , y: 0.0
      , theta: 0.0
      });
      var tessasOffice = new Location({
        name: 'Tessa\'s Office'
      , x: 0.0
      , y: 0.0
      , theta: 0.0
      });
      var brandonsOffice = new Location({
        name: 'Brandon\'s Office'
      , x: 0.0
      , y: 0.0
      , theta: 0.0
      });
      this.add([
        greenRoom
      , cafeteria
      , tessasOffice
      , brandonsOffice
      ]);
    }

  });

  return Locations;
});



