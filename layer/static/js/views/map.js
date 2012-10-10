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
              x = d3.scale.linear().domain([0, w])
              y = d3.scale.ordinal().domain([0, h]);

              var svg = d3.select("#floorplan-div").append("svg")
                .attr("width", w)
                .attr("height", h)
                .attr("id","damap");

              svg.append("svg:image")
                .attr("xlink:href", "/static/willow-floorplan.png")
                .attr("width", w)
                .attr("height", h);

            this.get('controller').get('content').addArrayObserver( this);
        },
        arrayWillChange: function() {},
        arrayDidChange: function() {
            this.drawRooms();
        },
        drawRooms: function() {
            var rooms = this.get('controller').get('content');
            if(!rooms) {
                return;
            }
            var svg = d3.select("#damap").selectAll(".room")
                .data(rooms.toArray())
                .enter().append("svg:rect")
                  .attr("class", "room")
                  .attr("x", function(d) { return d.get('map_x'); })
                  .attr("y", function(d) { return d.get('map_y'); })
                  .attr("width", function(d) { return d.get('map_width'); })
                  .attr("height", function(d) { return d.get('map_height'); })
                  .on("mouseover", function(d) {
                    d3.select("#placename").text(d.get('name'));
                    })
                  .on("mouseout", function() {
                    d3.select("#placename").text("");
                    })
                  .on("click", function(d) {
                    App.get('router').send("navigateTo",{
                            robot_id: App.router.get('robotController').get('content').get('id'),
                            place_id: d.get('id')
                        });
                    });
        }
    });

});
