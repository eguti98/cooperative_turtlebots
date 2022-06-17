#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from math import radians

import os

import numpy as np
from nav_msgs.msg import Odometry


class DrawALine:

    def __init__(self):

        rospy.init_node('drawaline', anonymous=True)
        robot = rospy.get_param("/robot_name")
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        r = rospy.Rate(10) #10HZ
        if robot == "tb3_0":
            # FORWARD
            move_cmd = Twist()
            move_cmd.linear.x = 0.20
        elif robot == "tb3_1":
            # FORWARD
            move_cmd = Twist()
            move_cmd.linear.x = 0.20
        elif robot == "tb3_2":
            # FORWARD
            move_cmd = Twist()
            move_cmd.linear.x = 0.10

        # STOP
        stop_cmd = Twist()
        stop_cmd.linear.x = 0
        stop_cmd.angular.z = 0
        # LEFT TURN
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(30)


        count = 0
        while not rospy.is_shutdown():
            if robot == "tb3_0":
                # FORWARD
                rospy.loginfo('Going Straight')
                for x in range(0, 500):
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
            elif robot == "tb3_1":
                # FORWARD
                rospy.loginfo('Going Straight')
                for x in range(0, 500):
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
            elif robot == "tb3_2":
                # FORWARD
                rospy.loginfo('Going Straight')
                for x in range(0, 1000):
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
        # STOP
            rospy.loginfo('Stopping')
            for x in range(0, 20):
                self.cmd_vel.publish(stop_cmd)
                r.sleep()
        # LEFT TURN
            rospy.loginfo('Turning')
            for x in range(0, 60):
                self.cmd_vel.publish(turn_cmd)
                r.sleep()
        # STOP
            rospy.loginfo('Stopping')
            for x in range(0, 20):
                self.cmd_vel.publish(stop_cmd)
                r.sleep()

            count = count + 1
            if count == 4:

                shutdown(self)
            if count == 0:
                rospy.loginfo("TurtleBot should be close to the original starting position (but it's probably way off)")

    def shutdown(self):

        # STOP
        rospy.loginfo('Stop Drawing Lines')
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':

    try:
        DrawALine()
    except:
        rospy.loginfo('node terminated.')
