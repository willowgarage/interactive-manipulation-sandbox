$(document).ready(function () {
	console.log("starting Mobile TurtleBot Interface");
	var ros = new ROS("ws://10.0.129.19:9090/");
	var cmd_vel = new ros.Topic({name:"/cmd_vel"});

	$("#forward").bind('tap',function(o){
		cmd_vel.publish({linear:{x:1.0,y:0,z:0},angular:{x:0,y:0,z:0}});
	});
	$("#backwards").bind('tap',function(o){
		cmd_vel.publish({linear:{x:-1.0,y:0,z:0},angular:{x:0,y:0,z:0}});
	});
	$("#left").bind('tap',function(o){
		cmd_vel.publish({linear:{x:0,y:0,z:0},angular:{x:0,y:0,z:-1}});
	});
	$("#right").bind('tap',function(o){
		cmd_vel.publish({linear:{x:0,y:0,z:0},angular:{x:0,y:0,z:1}});
	});
});
