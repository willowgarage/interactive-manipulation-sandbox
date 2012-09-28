define([
  'jquery'
, 'underscore'
, 'backbone'
, 'views/locations'
],
function(
  $
, _
, Backbone
, LocationsView
) {

  var AppView = Backbone.View.extend({

    initialize: function() {
      this.setElement('#app');
    }

  , render: function() {
      var locationsView = new LocationsView();
      this.$el.find('#content').html(locationsView.render().el);

      return this;
    }

  });

  return AppView;
});

