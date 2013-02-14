from threading import Thread

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


Publisher = rospy.Publisher  # TestPublisher


###############################################################################


class RobotRunner(Thread):

    SLEEP_INTERVAL = 0.1

    def __init__(self, publisher, twist, *args, **kwargs):
        """Setup a runner to start moving."""
        super(RobotRunner, self).__init__(*args, **kwargs)
        self.p = publisher
        self.twist = twist
        self.moving = False

    def run(self):
        self.moving = True
        while self.moving:
            logger.debug(' -- --- ---- PUBLISHING!!!')
            self.p.publish(self.twist)
            rospy.sleep(RobotRunner.SLEEP_INTERVAL)


class RobotProxy(object):

    MOVE_LINEAR_FACTOR = 1.0
    MOVE_ANGULAR_FACTOR = 2.0

    MOTIONS = {
        'move forward': lambda speed: Twist(linear=Vector3(
            RobotProxy.MOVE_LINEAR_FACTOR * speed, 0, 0)),
        'move back': lambda speed: Twist(linear=Vector3(
            0, -1.0 * RobotProxy.MOVE_LINEAR_FACTOR * speed, 0)),
        'turn left': lambda speed: Twist(angular=Vector3(
            0, 0, RobotProxy.MOVE_ANGULAR_FACTOR * speed)),
        'turn right': lambda speed: Twist(angular=Vector3(
            0, 0, -1.0 * RobotProxy.MOVE_ANGULAR_FACTOR * speed)),
    }

    SPEED_LIMITS = (1.0, 3.0)

    # TOPIC = 'base_controller/command'  # DEBUG: ADD BACK THE '/prl' PREFIX
    TOPIC = '/cmd_vel_mux/input/teleop'

    def __init__(self, speed=2.0):
        # Reuse the publisher by all threads, since it takes a while.
        rospy.init_node('move')
        self._publisher = Publisher(RobotProxy.TOPIC, Twist)
        self._speed = speed
        # Get the first runner ready.
        self.robot = RobotRunner(self._publisher, Twist())

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, speed):
        """Set the speed of the robot.

        Raises ValueError if speed is not between RobotProxy.SPEED_LIMITS.
        """
        low, high = RobotProxy.SPEED_LIMITS
        if speed < low or speed > high:
            raise ValueError('speed %s is out of desired range.' % speed)
        logger.info('Setting robot speed to %s.' % speed)
        self._speed = speed

    @property
    def moving(self):
        return self.robot.moving

    def stop_moving(self, callback=None):
        """Sends command to robot to stop moving.

        The call back, if provided, is called regardless of whether the robot
        was in fact moving or not.

        Positional arguments:
        callback -- a callable to be called once robot is stoped (optional).
        """
        if self.robot.moving:
            logger.debug("...stopping the robot...")

            # kill running move-thread.
            self.robot.moving = False
            rospy.sleep(RobotRunner.SLEEP_INTERVAL)

            # Send a "stop" twist to the robot.
            stop_twist = Twist()
            self._publisher.publish(stop_twist)

        if callable(callback):
            callback()

    def move(self, motion):
        """Sends command to robot to motion in some way.

        If the robot is moving, stop it before sending the new move command.

        Positional arguments:
        motion -- (string) one of RobotProxy.MOTIONS.
        """
        if self.robot.is_alive():
            # The robot is still moving. Wait and retry.
            self.stop_moving()
            rospy.sleep(RobotRunner.SLEEP_INTERVAL)
            if self.robot.is_alive:
                # wtf...
                raise RuntimeError('The robot is moving... why is it moving?')

        if motion not in RobotProxy.MOTIONS:
            raise ValueError('motion passed must be one of RobotProxy.MOTIONS')

        # Get a new runner.
        twist = RobotProxy.MOTIONS[motion](self.speed)
        self.robot = RobotRunner(self._publisher, twist)
        logger.debug('moving (%s)...' % motion)
        self.robot.start()
