/**
 * @author David Gossow <dgossow@willowgarage.com>
 */

THREE.InteractiveMarkerClient = function( rosbridgeURL, topicNS ) 
{
  THREE.Object3D.call( this );
  
  var ros = new ROS( rosbridgeURL );

  var topic = new ros.Topic({
    name : topicNS+'/tunneled',
    messageType : 'visualization_msgs/InteractiveMarkerUpdate'
  });

  var that = this;
  topic.subscribe( that.processUpdate.bind( that ) );
};

THREE.InteractiveMarkerClient.prototype = Object.create( THREE.Object3D.prototype );

THREE.InteractiveMarkerClient.prototype.processUpdate = function(message) 
{ 
  //console.log(message);
  var that=this;

  // delete markers
  message.erases.forEach(function(erase) 
  {
    that.remove( that.getChildByName(erase) );
  });

  // update poses
  message.poses.forEach(function(poseMsg) 
  {
    intMarkerObj = that.getChildByName( poseMsg.name );
    console.log("moving ",poseMsg.name);
    intMarkerObj.setPose( poseMsg.pose );
  });

  // add new markers
  message.markers.forEach(function(intMarkerMsg) 
  {
    var oldIntMarkerObj = that.getChildByName( intMarkerMsg.name );
    if ( oldIntMarkerObj != undefined )
    {
      console.log(that.add);
      that.remove( oldIntMarkerObj );
    }
    intMarkerObj = new THREE.InteractiveMarkerHelper( intMarkerMsg );
    //console.log("adding ",intMarkerObj);
    that.add(intMarkerObj);
  });
};
