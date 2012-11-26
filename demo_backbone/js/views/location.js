define([
  'jquery'
, 'underscore'
, 'backbone'
, 'models/location'
, 'text!templates/location.html'
, 'views/Video'
],
function(
  $
, _
, Backbone
, Location
, LocationTemplate
, VideoView
) {

  var LocationView = Backbone.View.extend({

    template: _.template(LocationTemplate)

  , initialize: function() {

    }

  , render: function() {
      var html = this.template({ 'model': this.model });
      this.$el.html(html);

      var videoView = new VideoView();
      this.$el.find('#video').html(videoView.render().el);

      return this;
    }

  });

  return LocationView;
});

