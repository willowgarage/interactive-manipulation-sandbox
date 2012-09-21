// Create a connection to the rosbridge WebSocket server.
var ros = new ROS('ws://localhost:9090');

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
  console.log(error);
});


var action = new Action({
  ros    : ros
, name   : 'NavigateTo'
});
action.inputs.x = 11.0;
action.inputs.y = 22.0;

action.on('status', function(status) {
  console.log(status);
  console.log('Status');
});

action.on('feedback', function(feedback) {
  console.log(feedback);
  console.log('Feedback');
});

action.on('result', function(result) {
  console.log(result);
  console.log('Finished');
});


