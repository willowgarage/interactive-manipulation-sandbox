define([
  'jquery'
, 'underscore'
, 'backbone'
, 'text!templates/video.html'
],
function(
  $
, _
, Backbone
, VideoTemplate
) {

  var VideoView = Backbone.View.extend({

    template: _.template(VideoTemplate)

  , initialize: function() {
    }

  , render: function() {
      var html = this.template({
        'url': 'http://localhost:8080/stream?topic=/camera/rgb/image_color'
      });
      this.$el.html(html);
      return this;
    }

  });

  return VideoView;
});

