#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from math import radians

import os

import numpy as np
from nav_msgs.msg import Odometry


class DrawASquare:

    def __init__(self):

        # initiliaze

        rospy.init_node('drawasquare', anonymous=True)

        # What to do you ctrl + c

        rospy.on_shutdown(self.shutdown)

        robot = rospy.get_param("/robot_name")

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # 10 HZ

        r = rospy.Rate(10)

    # create three different Twist() variables.  One for moving forward.  One for stopping. One for turning 45 degrees.

        # let's go forward at 0.2 m/s

        move_cmd = Twist()
        move_cmd.linear.x = 0.10

    # by default angular.z is 0 so setting this isn't required

        # let's stop

        stop_cmd = Twist()
        stop_cmd.linear.x = 0
        stop_cmd.angular.z = 0

        # let's turn at 45 deg/s

        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(30)  # 45 deg/s in radians/s

    # two keep drawing squares.  Go forward for 4 seconds (40 x 10 HZ) then turn for 2 second

        count = 0
        while not rospy.is_shutdown():

        # go forward 1 m (4 seconds * 0.25 m / seconds)

            rospy.loginfo('Going Straight')
            for x in range(0, 100):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

        # Stopping

            rospy.loginfo('Stopping')
            for x in range(0, 20):
                self.cmd_vel.publish(stop_cmd)
                r.sleep()

        # turn 90 degrees

            rospy.loginfo('Turning')
            for x in range(0, 30):
                self.cmd_vel.publish(turn_cmd)
                r.sleep()

        # Stopping

            rospy.loginfo('Stopping')
            for x in range(0, 20):
                self.cmd_vel.publish(stop_cmd)
                r.sleep()

            count = count + 1
            if count == 4:

                # count = 0

                shutdown(self)
            if count == 0:
                rospy.loginfo("TurtleBot should be close to the original starting position (but it's probably way off)"
                              )

    def shutdown(self):

        # stop turtlebot

        rospy.loginfo('Stop Drawing Squares')
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':

    try:
        DrawASquare()
    except:
        rospy.loginfo('node terminated.')
