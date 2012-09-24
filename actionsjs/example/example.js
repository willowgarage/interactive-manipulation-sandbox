// Create a connection to the rosbridge WebSocket server.
var ros = new ROS('ws://prl:9099');

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
  console.log(error);
});

ros.on('connection', function() {
  console.log('connection');
  var action = new Action({
    ros    : ros
  , name   : 'NavigateToPose'
  });
  action.inputs.x = 18.639;
  action.inputs.y = 22.715;
  action.inputs.theta = 0.0;
  action.inputs.frame_id = '/map';

  action.on('status', function(status) {
    console.log('Status');
    console.log(status);
  });

  action.on('feedback', function(feedback) {
    console.log('Feedback');
    console.log(feedback);
  });

  action.on('result', function(result) {
    console.log('Result');
    console.log(result);
  });

  console.log('Calling NavigateTo action');
  action.execute();
});


