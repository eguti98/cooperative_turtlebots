#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from math import radians

import os

import numpy as np
from nav_msgs.msg import Odometry

cov_zupt = int()

class DrawALine:

    # global cov_x
    # cov_x = 0

    def __init__(self):

        rospy.init_node('drawaline', anonymous=True)
        robot = rospy.get_param("robot_name")

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

        rospy.Subscriber("covariance",Float64MultiArray,self.covariance_callback)
        # self.covariance_subscriber()


        count = 0
        while not rospy.is_shutdown():


            if robot == "tb3_0":
                # FORWARD
                rospy.loginfo('Going Straight')
                for x in range(0, 250):
                    self.cmd_vel.publish(move_cmd)
                    # # PERIODIC ZUPT
                    # if (x == 50 or x == 100 or x == 150 or x == 200):
                    #     rospy.loginfo('Zupt')
                    #     for y in range(0,20):
                    #         self.cmd_vel.publish(stop_cmd)
                    #         r.sleep()
                    # COVARIANCE ZUPT
                    if cov_zupt == 1:
                        rospy.loginfo('Zupt')
                        for y in range(0,20):
                            self.cmd_vel.publish(stop_cmd)
                            r.sleep()
                        # cov_zupt = 0
                    r.sleep()

            elif robot == "tb3_1":
                # FORWARD
                rospy.loginfo('Going Straight')
                for x in range(0, 250):
                    self.cmd_vel.publish(move_cmd)
                    # # PERIODIC ZUPT
                    # if (x == 50 or x == 100 or x == 150 or x == 200):
                    #     rospy.loginfo('Zupt')
                    #     for y in range(0,20):
                    #         self.cmd_vel.publish(stop_cmd)
                    #         r.sleep()
                    # COVARIANCE ZUPT
                    if cov_zupt == 1:
                        rospy.loginfo('Zupt')
                        for y in range(0,20):
                            self.cmd_vel.publish(stop_cmd)
                            r.sleep()
                        # cov_zupt = 0
                    r.sleep()

            elif robot == "tb3_2":
                # FORWARD
                rospy.loginfo('Going Straight')
                for x in range(0, 500):
                    self.cmd_vel.publish(move_cmd)
                    # # PERIODIC ZUPT
                    # if (x == 100 or x == 200 or x == 300 or x == 400):
                    #     rospy.loginfo('Zupt')
                    #     for y in range(0,20):
                    #         self.cmd_vel.publish(stop_cmd)
                    #         r.sleep()
                    # print(cov_zupt)
                    # COVARIANCE ZUPT
                    if cov_zupt == 1:
                        rospy.loginfo('Zupt')
                        for y in range(0,20):
                            self.cmd_vel.publish(stop_cmd)
                            r.sleep()
                        # cov_zupt = 0
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

            if robot == "tb3_0":
                if count == 2:
                    shutdown(self)
            elif robot == "tb3_1":
                if count == 2:
                    shutdown(self)
            elif robot == "tb3_2":
                if count == 1:
                    shutdown(self)

            if count == 0:
                rospy.loginfo("TurtleBot should be close to the original starting position (but it's probably way off)")

    def shutdown(self):

        # STOP
        rospy.loginfo('Stop Drawing Lines')
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def covariance_callback(self,covariance):

        global cov_zupt

        cov_x = covariance.data[0]
        cov_y = covariance.data[1]
        cov_z = covariance.data[2]

        if (cov_x > 5 or cov_y > 5 or cov_z > 5):
            cov_zupt = 1
        else:
            cov_zupt = 0


if __name__ == '__main__':

    try:
        DrawALine()
    except:
        rospy.loginfo('node terminated.')
