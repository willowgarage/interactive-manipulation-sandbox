define([
    'ember',
    'app',
    'd3',
    'text!templates/map.handlebars'
],function(Ember,App,d3,mapHtml) {

  App.MapView = Ember.View.extend({
    template: Ember.Handlebars.compile(mapHtml),
    didInsertElement: function() {
      var w = 480,
        h = 374,
        x = d3.scale.linear().domain([0, w]),
        y = d3.scale.ordinal().domain([0, h]);

        var svg = d3.select("#floorplan-div").append("svg")
          .attr("width", w)
          .attr("height", h)
          .attr("id","mapsvg");

        svg.append("svg:image")
          .attr("xlink:href", "/static/images/willow-floorplan.png")
          .attr("width", w)
          .attr("height", h);

      this.get('controller').get('content').addArrayObserver( this);
    },
    arrayWillChange: function() {},
    arrayDidChange: function() {
      this.drawRooms();
    },
    drawRooms: function() {
      var places = this.get('controller').get('content');
      if(!places) {
          return;
      }
      var rooms = places.filter( function(p) { return !p.get('isOutlet'); });
      var outlets = places.filter( function(p) { return p.get('isOutlet'); });
      var map = d3.select("#mapsvg");


      function nodeSelected(d) {
        d3.select("#placename")
          .text(d.get('name'));
        _this.get('controller').set('placeId', d.get('id'));
      }

      /* Draw Rooms */
      var _this = this;
      map.selectAll(".room")
        .data(rooms.toArray())
        .enter().append("svg:rect")
          .attr("class", "room")
          .attr("x", function(d) { return d.get('map_x'); })
          .attr("y", function(d) { return d.get('map_y'); })
          .attr("width", function(d) { return d.get('map_width'); })
          .attr("height", function(d) { return d.get('map_height'); })
          .on("click", nodeSelected);

      /* Draw Outlets */
      map.selectAll(".outlet")
        .data(outlets.toArray())
        .enter().append("svg:image")
          .attr("class", "outlet")
          .attr("xlink:href", "/static/outlet.jpg")
          .attr("x", function(d) { return d.get('map_x'); })
          .attr("y", function(d) { return d.get('map_y'); })
          .attr("width", 20)
          .attr("height", 20)
          .on("click", nodeSelected);
    }
  });

});
