import gevent

from gevent import monkey
monkey.patch_all()

import rospy
from geometry_msgs.msg import Twist, Vector3

import logging
logger = logging.getLogger('robot')


class TestPublisher:
    """
    Mock the rospy.Publisher class.
    """

    def __init__(self, name, msg_class):
        self.name = name
        self.msg_class = msg_class

    def publish(self, msg):
        logger.debug('Mock Publisher -- publishing to %s.' % self.name)


Publisher = rospy.Publisher  # TestPublisher  # rospy.Publisher


###############################################################################


class RobotProxy:

    SLEEP_INTERVAL = 0.1

    MOVE_LINEAR_FACTOR = 0.15
    MOVE_ANGULAR_FACTOR = 0.5

    MOTIONS = {
        'move forward': lambda speed: Twist(linear=Vector3(
            RobotProxy.MOVE_LINEAR_FACTOR * speed, 0, 0)),
        'move back': lambda speed: Twist(linear=Vector3(
            -1.0 * RobotProxy.MOVE_LINEAR_FACTOR * speed, 0, 0)),
        'turn left': lambda speed: Twist(angular=Vector3(
            0, 0, RobotProxy.MOVE_ANGULAR_FACTOR * speed)),
        'turn right': lambda speed: Twist(angular=Vector3(
            0, 0, -1.0 * RobotProxy.MOVE_ANGULAR_FACTOR * speed)),
    }

    SPEED_LIMITS = (1.0, 3.0)

    # Turtlebot2's specific topic.
    TOPIC = '/cmd_vel_mux/input/teleop'

    @staticmethod
    def move_robot(publisher, twist):
        """Move the robot."""
        stop_twist = Twist()
        try:
            while True:
                publisher.publish(twist)
                logger.debug(' -- --- ---- PUBLISHING!!!')
                # We loose the concept of simulated time here.
                gevent.sleep(RobotProxy.SLEEP_INTERVAL)
        finally:
            # Send a "stop" twist to the robot immediately.
            publisher.publish(stop_twist)

    def __init__(self, speed=2.0):
        rospy.init_node('move')
        self._publisher = Publisher(RobotProxy.TOPIC, Twist)
        self._speed = speed
        self._movement = None

    @property
    def moving(self):
        return self._movement and not self._movement.ready()

    @property
    def speed(self):
        return self._speed

    def stop_moving(self, callback=None):
        """Sends command to robot to stop moving.

        The call back, if provided, is called regardless of whether the robot
        was in fact moving or not.

        Positional arguments:
        callback -- a callable to be called once robot is stoped (optional).
        """
        if self.moving:
            logger.debug("...stopping the robot...")
            self._movement.kill()
        if callable(callback):
            callback()

    def move(self, motion):
        """Sends command to robot to motion in some way.

        If the robot is moving, stop it before sending the new move command.

        Positional arguments:
        motion -- (string) one of RobotProxy.MOTIONS.
        """
        if self.moving:
            # The robot is still moving. Stop it.
            self._movement.kill()

        if motion not in RobotProxy.MOTIONS:
            raise ValueError('motion passed must be one of RobotProxy.MOTIONS')

        twist = RobotProxy.MOTIONS[motion](self.speed)
        logger.debug('moving (%s)...' % motion)
        self._movement = gevent.spawn(RobotProxy.move_robot,
                                      self._publisher, twist)
