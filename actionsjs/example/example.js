// ROS will be initialized after a host URI is set.
var ros = null;

// Handles form inputs.
window.onload = function() {

  // Connect to the rosbridge host when the user submits the URI.
  var hostDetailsForm = document.getElementById('host_details_form');
  hostDetailsForm.addEventListener('submit', function(e) {
    e.preventDefault(); // Do not submit the page
    var hostUri = document.getElementById('host_uri').value;
    startRos(hostUri);
  });

  // Send coordinates to the NavigateToPose action.
  var navigateToForm = document.getElementById('navigate_to_form');
  navigateToForm.addEventListener('submit', function(e) {
    e.preventDefault(); // Do not submit the page
    var x = document.getElementById('navigate_to_x').value;
    var y = document.getElementById('navigate_to_y').value;
    var theta = document.getElementById('navigate_to_theta').value;
    navigateTo(x, y, theta);
  });
};

// Opens a connection to the ROS server.
function startRos(uri) {
  ros = new ROS(uri);
  var connectionStatusElement = document.getElementById('connection_status');
  connectionStatusElement.innerHtml = 'Connecting...';
  document.getElementById('navigate_to_submit').disabled = false;

  ros.on('connection', function() {
    connectionStatusElement.innerHTML = 'Connected';
    document.getElementById('navigate_to_submit').disabled = false;
  });

  ros.on('close', function() {
    connectionStatusElement.innerHTML = 'Disconnected';
    document.getElementById('navigate_to_submit').disabled = true;
  });
};

// Performs the "NavigateToPose" action primitive.
function navigateTo(x, y, theta) {
  // NavigateToPose is the name of the action primitive on the robot.
  var action = new Action({
    ros  : ros
  , name : 'NavigateToPose'
  });

  action.inputs.x = x;
  action.inputs.y = y;
  action.inputs.theta = theta;
  action.inputs.frame_id = '/map';

  action.on('feedback', function(feedback) {
    document.getElementById('navigation_feedback').innerHTML = feedback;
  });

  // Updates from the robot on every status change (for example, a movement)
  action.on('status', function(status) {
    document.getElementById('navigation_status').innerHTML = status;
  });

  // Result is called when action finished. The action may not have succeeded
  // though, for example, if there was an obstacle at the location.
  action.on('result', function(result) {
    document.getElementById('navigation_result').innerHTML = result;
  });

  action.execute();
};


