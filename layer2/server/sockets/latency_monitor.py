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

ALTERNATIVE IMPLEMENTATION:
Move all this logic to the client. Let the server only bounce the packets and
feed from the results in them.
"""

import math
import random
from datetime import datetime, timedelta
from dateutil import parser
import gevent
from socketio.namespace import BaseNamespace

import collections


class RTTComputingStrategy(object):
    """Base class for all latency computing strategies."""

    RTT_WORST = 0.0
    RTT_BEST = 0.0

    def __init__(self, rtt=0.0, relative_rtt=0.0, best=RTT_BEST,
                 worst=RTT_WORST):
        self.best = best
        self.worst = worst

        # A float with the absolute latency, in seconds.
        self.rtt = rtt
        # A number between 0 and 1, to use as reference reltive to the expected
        # BEST/WORST values.
        self.relative_rtt = relative_rtt

    def feed(self, delta):
        """Receive a new time delta and recompute the RTT.

        This implementation drops the previuosly computed value.
        """
        raise NotImplementedError()


class StatelessRTT(RTTComputingStrategy):
    """
    Latency computing strategy.

    Use last feedback to produce the current latecy. Previuos values do not
    alter computation.
    """

    def feed(self, delta):
        self.rtt = delta
        self.relative_rtt = (self.rtt - self.best) / (self.worst - self.best)


class RunningAverageRTT(RTTComputingStrategy):
    """
    Latency computing strategy.

    Use last N deltas to produce the current latecy.
    """
    LENGTH = 10

    def __init__(self, *args, **kwargs):
        super(RunningAverageRTT, self).__init__(*args, **kwargs)

        # The average will be computed from this values.
        self.deltas = collections.deque(maxlen=self.LENGTH)

    def feed(self, delta):
        """Receive a new time delta and recompute the RTT.

        The current RTT is the running length of the LENGTH previuosly computed
        values.
        """
        self.deltas.append(delta)

        self.rtt = sum(self.deltas) / len(self.deltas)
        self.relative_rtt = (self.rtt - self.best) / (self.worst - self.best)


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
    RTT_BEST = 0.0  # seconds
    # Worst value acceptable: tuned to application.
    RTT_WORST = 3.0  # seconds.

    # Time between monitor packets sent, in seconds.
    HEALTH_CHECK_INTERVAL = 2.0  # seconds

    strategy = RunningAverageRTT

    def initialize(self):
        # Strategy for computing the RTT.
        self._rtt_computation = self.strategy(rtt=LatencyMonitor.RTT_BEST,
                                              relative_rtt=0.0,
                                              worst=LatencyMonitor.RTT_WORST,
                                              best=LatencyMonitor.RTT_BEST)

    @property
    def _rtt(self):
        return self._rtt_computation.rtt

    @property
    def _relative_rtt(self):
        return self._rtt_computation.relative_rtt

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
                'rtt': self._rtt_computation.rtt,
                'relative_rtt': self._rtt_computation.relative_rtt,
                }

            self.emit('health check', health_monitor_packet)

    def recv_connect(self):
        """Setup the monitoring mechanism."""
	gevent.spawn(self._monitor_packet_gun)

    def on_bounced_health_check(self, health_monitor_packet):
        # Only piece of the bounced packet of interest to the server.
        timestamp = health_monitor_packet['timestamp']

        hidrated_timestamp = parser.parse(timestamp)
        delta = datetime.now() - hidrated_timestamp
        self._rtt_computation.feed(delta.total_seconds())
