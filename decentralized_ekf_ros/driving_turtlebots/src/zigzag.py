#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from math import radians

import os

import numpy as np
from nav_msgs.msg import Odometry


class DrawAZigzag:

    def __init__(self):

        rospy.init_node('drawazigzag', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        r = rospy.Rate(10) #10HZ


        # FORWARD
        move_cmd = Twist()
        move_cmd.linear.x = 0.20
        # STOP
        stop_cmd = Twist()
        stop_cmd.linear.x = 0
        stop_cmd.angular.z = 0
        # LEFT TURN
        turn_left_cmd = Twist()
        turn_left_cmd.linear.x = 0
        turn_left_cmd.angular.z = radians(15)
        # RIGHT TURN
        turn_right_cmd = Twist()
        turn_right_cmd.linear.x = 0
        turn_right_cmd.angular.z = -radians(15)

        count = 0
        while not rospy.is_shutdown():

            if count == 0:
                # LEFT TURN
                rospy.loginfo('Left Turning')
                for x in range(0, 30):
                    self.cmd_vel.publish(turn_left_cmd)
                    r.sleep()
                # STOP
                rospy.loginfo('Stopping')
                for x in range(0, 30):
                    self.cmd_vel.publish(stop_cmd)
                    r.sleep()
                # FORWARD
                rospy.loginfo('Going Straight')
                for x in range(0, 70):
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                # STOP
                rospy.loginfo('Stopping')
                for x in range(0, 30):
                    self.cmd_vel.publish(stop_cmd)
                    r.sleep()
        # RIGHT TURN
            rospy.loginfo('Right Turning')
            for x in range(0, 60):
                self.cmd_vel.publish(turn_right_cmd)
                r.sleep()
        # STOP
            rospy.loginfo('Stopping')
            for x in range(0, 30):
                self.cmd_vel.publish(stop_cmd)
                r.sleep()
        # FORWARD
            rospy.loginfo('Going Straight')
            for x in range(0, 140):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
        # STOP
            rospy.loginfo('Stopping')
            for x in range(0, 30):
                self.cmd_vel.publish(stop_cmd)
                r.sleep()
        # LEFT TURN
            rospy.loginfo('Left Turning')
            for x in range(0, 60):
                self.cmd_vel.publish(turn_left_cmd)
                r.sleep()
        # STOP
            rospy.loginfo('Stopping')
            for x in range(0, 30):
                self.cmd_vel.publish(stop_cmd)
                r.sleep()
        # FORWARD
            rospy.loginfo('Going Straight')
            for x in range(0, 140):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
        # STOP
            rospy.loginfo('Stopping')
            for x in range(0, 30):
                self.cmd_vel.publish(stop_cmd)
                r.sleep()

            count = count + 1
            if count == 2:

                shutdown(self)
            if count == 0:
                rospy.loginfo("TurtleBot should be close to the original starting position (but it's probably way off)")

    def shutdown(self):

        # STOP
        rospy.loginfo('Stop Drawing Zigzags')
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':

    try:
        DrawAZigzag()
    except:
        rospy.loginfo('node terminated.')
