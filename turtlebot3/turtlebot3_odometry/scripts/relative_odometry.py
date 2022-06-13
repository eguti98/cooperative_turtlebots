#!/usr/bin/env python3
import roslib
import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

curr_odom_tb1 = Odometry()
curr_odom_tb2 = Odometry()
curr_odom_tb3 = Odometry()

def odometryCb_tb1(msg):     # Callback Function for position tb_1

    global curr_odom_tb1
    curr_odom_tb1 = msg
    #print("Tb1", msg.pose.pose.position)
#print("Tb 1 Position:\n", curr_odom_tb1.pose.pose.position)

def odometryCb_tb2(msg):     # Callback Function for position tb_2

    global curr_odom_tb2
    curr_odom_tb2 = msg
    #print("Tb2",  msg.pose.pose.position)
#print("Tb 2 Position:\n", curr_odom_tb2.pose.pose.position)

def odometryCb_tb3(msg):     # Callback Function for position tb_3

    global curr_odom_tb3
    curr_odom_tb3 = msg
    #print("Tb3",  msg.pose.pose.Position)
#print("Tb 3 Postiion:\n", curr_odom_tb3.pose.pose.position)

if __name__ == "__main__":

    rospy.init_node('relative_odom', anonymous=True)     # Make Node
    rate = rospy.Rate(0.5)     # Frequency in Hz

    rospy.Subscriber('/tb3_0/odom',Odometry,odometryCb_tb1)     # Subscribe to Odometry tb_1
    rospy.Subscriber('/tb3_1/odom',Odometry,odometryCb_tb2)     # Subscribe to Odometry tb_2
    rospy.Subscriber('/tb3_2/odom',Odometry,odometryCb_tb3)     # Subscribe to Odometry tb_3

    while (not rospy.is_shutdown()):

        rate.sleep()

        range_tb1_tb2 = Range()
        range_tb1_tb2.range = math.sqrt((curr_odom_tb1.pose.pose.position.x-curr_odom_tb2.pose.pose.position.x)**2+(curr_odom_tb1.pose.pose.position.y-curr_odom_tb2.pose.pose.position.y)**2)
        #print("Distance from Turtlebot 1 to Turtlebot 2:", range_tb1_tb2.range)
        pub1 = rospy.Publisher('range_tb1_tb2', Range, queue_size=100)     # Publish Topic with Distance tb1_tb2
        #rospy.loginfo(range_tb1_tb2)
        pub1.publish(range_tb1_tb2)

        range_tb1_tb3 = Range()
        range_tb1_tb3.range = math.sqrt((curr_odom_tb1.pose.pose.position.x-curr_odom_tb3.pose.pose.position.x)**2+(curr_odom_tb1.pose.pose.position.y-curr_odom_tb3.pose.pose.position.y)**2)
        #print("Distance from Turtlebot 1 to Turtlebot 3:", range_tb1_tb3.range)
        pub2 = rospy.Publisher('range_tb1_tb3', Range, queue_size=100)     # Publish Topic with Distance tb1_tb3
        #rospy.loginfo(range_tb1_tb3)
        pub2.publish(range_tb1_tb3)

        range_tb2_tb3 = Range()
        range_tb2_tb3.range = math.sqrt((curr_odom_tb2.pose.pose.position.x-curr_odom_tb3.pose.pose.position.x)**2+(curr_odom_tb2.pose.pose.position.y-curr_odom_tb3.pose.pose.position.y)**2)
        #print("Distance from Turtlebot 2 to Turtlebot 3:", range_tb2_tb3)
        pub3 = rospy.Publisher('range_tb2_tb3', Range, queue_size=100)     # Publish Topic with Distance tb2_tb3
        #rospy.loginfo(range_tb2_tb3)
        pub3.publish(range_tb2_tb3)

    rospy.spin()
