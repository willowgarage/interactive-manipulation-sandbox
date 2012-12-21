"""
Monitoring a connection involves a constant, albeit light, traffic between
client and server. The current implementation uses a dedicated namespace in the
server and a dedicated socket in the client for this traffic. This scheme will
result in measurements of the round trip time (RTT) that reflects the best
possible value the real RTT will have for this particular client.

To implement monitoring a greenlet is spawn in server code, responsible for
emiting the "health monitor packet" every "health check interval"
milliseconds. Upong receiving the packet, the client will inmediatly bounce it
back to the server. The server receives the bounced packet by way of a "on_*"
or "recv_*" function.

----

The entire point is to measure RTT for each pair of packets, the sent and the
bounced. Sholud packets which arrive out of order should still affecet RTT
approximation? If not, a "sequence number" could employed to force order
in the stream of bounced packets.

ALTERNATIVE IMPLEMENTATION:
Use the callback function in the server side emit() to trigger a closure
wrapping both the datetime of the emitted packet, and the current approximation
for RTT.
This implementation involves less client work, and potentially less impact on
the network, since the client needs only call the "ack" function to trigger an
ACK packet back to the server.
"""

import math
import random
from datetime import datetime, timedelta
from dateutil import parser
import gevent
from socketio.namespace import BaseNamespace


class LatencyMonitor(BaseNamespace):
    """
    Ping-like packet going back and forth between client and server,
    measuring round trip time in real time.

    The algorithm to approximate the RTT is very poor right now: it does not
    considers previuos values, only considering last health check with respect
    to the reference values.
    """

    # Reference values for RTT aproximation:
    # Optimum value: no latency.
    RTT_BEST = timedelta()
    # Worst value acceptable: tuned to application.
    RTT_WORST = timedelta(0,   # days.
                          15,  # seconds.
                          0)   # microseconds.

    # Time between monitor packets sent, in seconds.
    HEALTH_CHECK_INTERVAL = 2.0;

    def initialize(self):
        # A timedelta object with the absolute latency.
        self._rtt = LatencyMonitor.RTT_BEST
        # A number between 0 and 1, to use as reference reltive to the expected
        # BEST/WORST values.
        self._relative_rtt = 0

    def _monitor_packet_gun(self):
        """Send a monitor packet every HEALTH_CHECK_INTERVAL.

        The anatomy of a health monitoring packet is simple:
        {
            "type": "health check",
            "timestamp": string formatted datetime object,
            "rtt": a float for the number of seconds,
            "relative_rtt": a float between 0 and 1
        }
        """
        while True:
            gevent.sleep(LatencyMonitor.HEALTH_CHECK_INTERVAL)  # yield.

            timestamp = unicode(datetime.now())
            health_monitor_packet = {
                'type': 'health check',
                'timestamp': timestamp,
                'rtt': self._rtt.total_seconds(),
                'relative_rtt': self._relative_rtt,
                }

            self.emit('health check', health_monitor_packet)

    def recv_connect(self):
        """Setup the monitoring mechanism."""
	gevent.spawn(self._monitor_packet_gun)

    def on_bounced_health_check(self, health_monitor_packet):
        # Only piece of the bounced packet of interest to the server.
        timestamp = health_monitor_packet['timestamp']

        hidrated_timestamp = parser.parse(timestamp)

        absolute_latency = datetime.now() - hidrated_timestamp
        relative_referece = LatencyMonitor.RTT_WORST - LatencyMonitor.RTT_BEST

        self._relative_rtt = (
            (absolute_latency - LatencyMonitor.RTT_BEST).total_seconds()
            / relative_referece.total_seconds())
        self._rtt = absolute_latency

        print "RTT:", self._rtt, ", relative reference:", self._relative_rtt  # DEBUG
