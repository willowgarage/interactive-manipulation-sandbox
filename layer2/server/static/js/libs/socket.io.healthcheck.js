define([
  'socketio'
],
function(
  io
) {

  /**
   * The health check namespace.
   *
   * @api public.
   */

  var HealthCheck = {};

  /**
   * Configuration affecting all extended sockets.
   *
   * @api private.
   */

  HealthCheck.config = {
    interval: 2000, // milliseconds between health checks.
  };

  /**
   * Data shared by all extended sockets.
   *
   * @api private.
   */

  HealthCheck.data = {
  };

  /**
   * RTT computing algorithm.
   *
   * @api private
   */

  HealthCheck.computeRTT = function(healthCheckData, delta){
    // Simplest possible thing.
    healthCheckData.rtt = delta;
  };

  /**
   * Update health data for a single socket.
   *
   * @api private
   */

  HealthCheck.update = function(healthCheckData, delta){
    // Runs all computations... currently just RTT.
    HealthCheck.computeRTT(healthCheckData, delta);
  };

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
  HealthCheck.extend = function(socket, onHealthCheck){

    // Connection health related data exclusive to this sockets.
    socket.healthCheck = {
      data: {rtt: 0}
    };

    socket.on('connect', function(){
      // Initiate the health check routine, sending the first health check packet.
      socket.emit('health check', {rtt: 0, timestamp: (new Date).getTime()});
    });

    // Handle server responses to health checks.
    socket.on('health check', function(healthCheckPacket){
      // Immediately compute the time delta.
      var delta = (new Date).getTime() - healthCheckPacket.timestamp;

      HealthCheck.update(socket.healthCheck.data, delta);

      // Expose the health data to the application.
      var healthCheckData = {rtt: socket.healthCheck.data.rtt};
      onHealthCheck(healthCheckData);
    });

    // Send health checks to the server at intervals.
    socket.healthCheck._interval = setInterval(function(){
      // Expose the health data to the server.
      var healthCheckData = {
        rtt: socket.healthCheck.data.rtt,
        timestamp: (new Date).getTime()
      };
      socket.emit('health check', healthCheckData);
    }, HealthCheck.config.interval);

    // Return the extended socket object.
    return socket;
  };

  return HealthCheck;
});
