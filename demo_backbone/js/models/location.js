define([
  'backbone'
],
function(
  Backbone
) {

  var Location = Backbone.Model.extend({
    idAttribute: 'name'

  , navigateTo: function() {
      // Navigate to the location using ros
      // Can get the x, y, and theta using this.get('x');
    }
  });

  return Location;
});


