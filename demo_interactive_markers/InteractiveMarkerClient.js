/**
 * @author David Gossow <dgossow@willowgarage.com>
 */

THREE.InteractiveMarkerClient = function( rosbridgeURL, topicNS ) 
{
  THREE.Object3D.call( this );
  
  var ros = new ROS( rosbridgeURL );

  var topic = new ros.Topic({
    name : topicNS+'/update_full',
    messageType : 'visualization_msgs/InteractiveMarkerInit'
  });

  var that = this;
  topic.subscribe( function(message){ return( that.processInit( message ) ); } );

  this.rootNode = new THREE.Object3D;
  this.add(this.rootNode);
};

THREE.InteractiveMarkerClient.prototype = Object.create( THREE.Object3D.prototype );


THREE.InteractiveMarkerClient.prototype.processInit = function(message) 
{ 
  console.log(this);
  this.remove(this.rootNode);
  this.rootNode = new THREE.Object3D;
  this.add(this.rootNode);
  
  var that=this;
  
  message.markers.forEach(function(intMarkerMsg) 
  {
    intMarkerObj = new THREE.InteractiveMarkerHelper( intMarkerMsg );
    that.rootNode.add(intMarkerObj);
  });
};
