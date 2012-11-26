define([
  'jquery'
, 'underscore'
, 'backbone'
, 'collections/locations'
, 'text!templates/locations.html'
, 'views/location'
],
function(
  $
, _
, Backbone
, Locations
, LocationsTemplate
, LocationView
) {

  var LocationsView = Backbone.View.extend({

    template: _.template(LocationsTemplate)

  , events: {
      'submit #locations_form': 'selectLocation'
    }

  , initialize: function() {
      this.collection = new Locations();
      this.collection.fetch();
    }

  , render: function() {
      var html = this.template({ 'locations': this.collection });
      this.$el.html(html);
      return this;
    }

  , selectLocation: function(event) {
      event.preventDefault();

      var locationId = $('#locations_selector').val();
      var location = this.collection.get(locationId);

      // THERE ARE BETTER WAYS TO CHANGE VIEWS IN BACKBONE
      // One example is the location could emit an event and another view
      // handler listens for that event and changes out the views (like
      // views/App). There's also a concept of routes that are not currently
      // implemented.
      var locationView = new LocationView({ model: location });
      $('#content').html(locationView.render().el);
      location.navigateTo();
    }

  });

  return LocationsView;
});

