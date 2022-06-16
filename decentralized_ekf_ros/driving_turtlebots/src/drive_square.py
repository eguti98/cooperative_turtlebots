#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry

class mob_robotics_lab:
    def __init__(self):
        rospy.loginfo("Driving Turtlebot Node Initialized")
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.square()

    def square(self):
        initial_time = rospy.get_time()
        twist = Twist()

        while not rospy.is_shutdown():
            twist.linear.x  = 0.1;      	twist.angular.z = 0.0;
            seconds = rospy.get_time()-initial_time

            self.pub.publish(twist)
            self.msg = rospy.wait_for_message("odom", Odometry)

            rospy.loginfo('%s', self.msg)
            if seconds > 5 and seconds < 8:
                rospy.loginfo("%d",seconds)
                twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0


                self.pub.publish(twist)
                self.msg = rospy.wait_for_message("odom", Odometry)
           # if seconds > 9 and seconds < 11:
           #     rospy.loginfo("%d",seconds)
           #     twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
           #     twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = np.pi


           #     self.pub.publish(twist)
           #     self.msg = rospy.wait_for_message("odom", Odometry)

           # if seconds > 12:
           #     rospy.loginfo("%d",seconds)
           #     twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
           #     twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0


           #     self.pub.publish(twist)
           #     self.msg = rospy.wait_for_message("odom", Odometry)

                rospy.sleep(1)
                rospy.signal_shutdown("Square Driven")




def main():

	rospy.init_node('driving_turtlebots_lab', anonymous=True)

	try:
		driving_turtlebots_lab = driving_turtlebots_lab()
	except rospy.ROSInterruptException:
		pass #Print Error

if __name__ == '__main__':
	main()
