#!/usr/bin/env python
from math import pi, sin, cos
import threading

from bluepy.btle import BTLEException
import KMControllers
from pose import Pose

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Twist,
    TransformStamped,
    Quaternion
)
import tf

class KeiganMotorDriver():
    """
    The Keigan Motor driver object.

    """
    
    def __init__(self):
        """
        
        """

        ## params
        # tf frames
        self._odom_frame = rospy.get_param('~odom_frame', 'odom')
        self._base_frame = rospy.get_param('~base_frame', 'base_footprint')
        # 2 wheel dolly's wheel separation is ~30.5 cm
        self._wheel_separation = float(rospy.get_param('~wheel_separation', 0.305))
        # 2 wheel dolly's wheel diameter is ~12 cm
        self._wheel_diameter = float(rospy.get_param('~wheel_diameter', 0.12))
        # wheel directions
        self._left_wheel_forward = rospy.get_param('~left_wheel_forward', True)
        self._right_wheel_forward = rospy.get_param('~right_wheel_forward', False)
        # keigan motors BLE mac addresses
        self._left_wheel_mac = rospy.get_param('~left_wheel_mac', 'D8:BA:37:4A:88:A1')
        self._right_wheel_mac = rospy.get_param('~right_wheel_mac', 'DD:7A:96:3C:E1:51')
        # update rate, default 30 Hz
        self._update_rate = rospy.get_param('~update_rate', 30.0)
        # lock for synchronizing between read and write to BLE
        self._lock = threading.Lock()

        ## vars
        self._pose = Pose()
        self._left_last_pos = 0.0
        self._right_last_pos = 0.0
        self._last_time = rospy.Time.now()
        # max rotation is 250 rpm, or ~26 rad/s
        self._max_rot = 26.0 #rad/s
        # wheels BLE device handle
        self._left_wheel_dev = KMControllers.BLEController(self._left_wheel_mac)
        self._right_wheel_dev = KMControllers.BLEController(self._right_wheel_mac)

        ## tf
        self._tfb = tf.TransformBroadcaster()

        ## pubs
        self._odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)

        ## subs
        self._twist_sub = rospy.Subscriber('cmd_vel', Twist, self._twist_callback, queue_size=1)

    def _twist_callback(self, cmd):
        """
        Set commanded velocities from Twist message.

        The commands are actually send/set during run loop, so delay
        is in worst case up to 1 / update_rate seconds.

        :type   cmd:    Twist
        :param  cmd:    The commanded velocities.

        """

        # compute differential wheel velocities
        self._left_wheel_vel = cmd.linear.x - (cmd.angular.z * self._wheel_separation * 0.5)
        self._right_wheel_vel = cmd.linear.x + (cmd.angular.z * self._wheel_separation * 0.5)
        # calculate wheel rate in rad/s
        left_rate = (self._left_wheel_vel / (self._wheel_diameter * 0.5)) * (1.0 if self._left_wheel_forward else -1.0)
        right_rate = (self._right_wheel_vel / (self._wheel_diameter * 0.5)) * (1.0 if self._right_wheel_forward else -1.0)
        
        # acquire lock to both wheels
        self._lock.acquire()

        # set the wheel speeds
        self._left_wheel_dev.speed(abs(left_rate))
        self._right_wheel_dev.speed(abs(right_rate))
        # rotate the motor according to the wheel rates sign
        if left_rate >= 0 :
            self._left_wheel_dev.runForward()
        else:
            self._left_wheel_dev.runReverse()
        if right_rate >= 0 :
            self._right_wheel_dev.runForward()
        else:
            self._right_wheel_dev.runReverse()
        
        # release lock to both wheels
        self._lock.release()
    
    def _update_pose(self):
        # acquire lock to both wheels
        self._lock.acquire()

        # get wheels' position measurements
        left_pos, _, _ = self._left_wheel_dev.read_motor_measurement()
        right_pos, _, _ = self._right_wheel_dev.read_motor_measurement()
        left_pos *= 1.0 if self._left_wheel_forward else -1.0
        right_pos *= 1.0 if self._right_wheel_forward else -1.0
        now = rospy.Time.now()
        
        # release lock to both wheels
        self._lock.release()

        # calculate pose from wheels' position measurements
        left_travel = (left_pos - self._left_last_pos) * self._wheel_diameter * 0.5
        right_travel = (right_pos - self._right_last_pos) * self._wheel_diameter * 0.5
        delta_time = (now - self._last_time).to_sec()

        delta_travel = (right_travel + left_travel) / 2
        delta_theta = (right_travel - left_travel) / self._wheel_separation

        if right_travel == left_travel:
            deltaX = left_travel*cos(self._pose.theta)
            deltaY = left_travel*sin(self._pose.theta)
        else:
            radius = self._wheel_separation/2 \
                * (right_travel + left_travel) / (right_travel - left_travel)

            # Find the instantaneous center of curvature (ICC).
            iccX = self._pose.x - radius*sin(self._pose.theta)
            iccY = self._pose.y + radius*cos(self._pose.theta)

            deltaX = cos(delta_theta)*(self._pose.x - iccX) \
                - sin(delta_theta)*(self._pose.y - iccY) \
                + iccX - self._pose.x
        
            deltaY = sin(delta_theta)*(self._pose.x - iccX) \
                + cos(delta_theta)*(self._pose.y - iccY) \
                + iccY - self._pose.y
        
        # update pose
        self._pose.x += deltaX
        self._pose.y += deltaY
        self._pose.theta = (self._pose.theta + delta_theta) % (2*pi)
        self._pose.xVel = delta_travel / delta_time
        self._pose.yVel = 0
        self._pose.thetaVel = delta_theta / delta_time

        self._last_time = now
        self._left_last_pos = left_pos
        self._right_last_pos = right_pos

    def _publish_tf(self):
        """
        Publish all tf frames.

        """
        # update robot pose from wheel measurements
        self._update_pose()

        now = rospy.Time.now()

        # publish odom tf
        self._tfb.sendTransform(
            (self._pose.x, self._pose.y, 0),
            (0, 0, sin(self._pose.theta / 2), cos(self._pose.theta / 2)),
            now,
            self._base_frame,
            self._odom_frame
        )

    def _publish_odometry(self):
        """
        Publish current pose as Odometry message.

        """
        # only publish if we have a subscriber
        if self._odom_pub.get_num_connections() == 0:
            return

        now = rospy.Time.now()

        # publish odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._pose.x
        odom.pose.pose.position.y = self._pose.y
        q = Quaternion()
        q.x = 0
        q.y = 0
        q.z = sin(self._pose.theta / 2)
        q.w = cos(self._pose.theta / 2)
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self._pose.xVel
        odom.twist.twist.angular.z = self._pose.thetaVel
        self._odom_pub.publish(odom)

    def run(self):
        """
        Publish data continuously with given rate.

        :type   update_rate:    int
        :param  update_rate:    The update rate.

        """
        # set the curve type to none
        self._left_wheel_dev.curveType(0)
        self._right_wheel_dev.curveType(0)
        # reset the measurements position to origin
        self._left_wheel_dev.presetPosition(0.0)
        self._right_wheel_dev.presetPosition(0.0)
        # enable motors
        self._left_wheel_dev.enable()
        self._right_wheel_dev.enable()
        # disable IMU to save battery
        self._left_wheel_dev.disableIMU()
        self._right_wheel_dev.disableIMU()
        # main loop
        r = rospy.Rate(self._update_rate)
        while not rospy.is_shutdown():
            self._publish_tf()
            self._publish_odometry()
            # sleep
            r.sleep()
    
    def on_shutdown(self):
        # disconnect all keigan motors
        self._left_wheel_dev.disconnect()
        self._right_wheel_dev.disconnect()

if __name__ == '__main__':
    rospy.init_node('keigan_motor_driver')
    keigan_motor_driver = KeiganMotorDriver()
    rospy.on_shutdown(keigan_motor_driver.on_shutdown)
    try:
        keigan_motor_driver.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
