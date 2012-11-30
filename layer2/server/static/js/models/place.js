define([
  'ember',
  'emberdata',
  'app'
],
function(
  Ember,
  DS,
  App
) {

  App.Place = DS.Model.extend({
    name: DS.attr('string'),
    description: DS.attr('string'),
    tags: DS.attr('string'),
    image: DS.attr('string'),
    pose_x: DS.attr('number'),
    pose_y: DS.attr('number'),
    pose_angle: DS.attr('number'),
    map_x: DS.attr('number'),
    map_y: DS.attr('number'),
    map_width: DS.attr('number'),
    map_height: DS.attr('number'),
    isOutlet: function() {
      return (this.get('tags') == "outlet");
    }.property(),
    isTable: function() {
      return (this.get('tags') == "table");
    }.property()
  });
});

