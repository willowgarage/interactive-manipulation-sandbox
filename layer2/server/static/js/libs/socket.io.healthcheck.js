define([
  'socketio'
],
function(
  io
) {
  /**
   * The healthCheck namespace.
   */
  var healthCheck = {
    // configuration affecting all extended sockets.
    config: {
      interval: 2000, // milliseconds
    },

    // per-socket state.
    of: []
  }

  /**
   * Extend a connected socket to provide connection health information.
   *
   * For now, provide the functionality as a function to extend an SocketNamespace object.
   *
   * The callback passed will receive as only argument the health check data object.
   *
   * @param {io.SocketNamespace} [socket] the socket object to extend.
   * @param {Function} [onHealthCheck] Called back upon a health check update.
   * @returns {io.SocketNamespace}
   * @api public
   */
  healthCheck.extend = function(socket, onHealthCheck){

    // Register a state entry for this socket.
    healthCheck.of[socket.name] = {rtt: 0};

    socket.on('connect', function(){
      // Initiate the health check routine, sending the first health check packet.
      socket.emit('health check', {rtt: 0, timestamp: (new Date).getTime()});
    });

    // Handle server responses to health checks.
    socket.on('health check', function(healthCheckPacket){
      var delta = (new Date).getTime() - healthCheckPacket.timestamp;

      // Compute the RTT using the delta in milliseconds.
      //
      // For now, simplest possible algorithm.
      healthCheck.of[socket.name].rtt = delta;

      // Expose the health data to the application.
      var healthCheckData = {rtt: healthCheck.of[socket.name].rtt};
      onHealthCheck(healthCheckData);
    });

    // Send health checks to the server at intervals.
    healthCheck.of[socket.name]._interval = setInterval(function(){
      // Expose the health data to the server.
      var healthCheckData = {
        rtt: healthCheck.of[socket.name].rtt,
        timestamp: (new Date).getTime()
      };
      socket.emit('health check', healthCheckData);
      }, healthCheck.config.interval);

    // Return the extended socket object.
    return socket;
  };

  return healthCheck;
});
