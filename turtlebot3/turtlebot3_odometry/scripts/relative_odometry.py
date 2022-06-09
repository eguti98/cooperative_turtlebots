#!/usr/bin/env python3
import roslib
import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

curr_odom_tb1 = Odometry()
curr_odom_tb2 = Odometry()
curr_odom_tb3 = Odometry()
# curr_odom_tb1_tr = Odometry()
# curr_odom_tb2_tr = Odometry()
# curr_odom_tb3_tr = Odometry()

def odometryCb_tb1(msg):     # Callback Function for position tb_1

    global curr_odom_tb1
    curr_odom_tb1 = msg
    #print("Tb1", msg.pose.pose.position)
    # print("Tb 1 Position:\n", curr_odom_tb1.pose.pose.position)

def odometryCb_tb2(msg):     # Callback Function for position tb_2

    global curr_odom_tb2
    curr_odom_tb2 = msg
    #print("Tb2",  msg.pose.pose.position)
    # print("Tb 2 Position:\n", curr_odom_tb2.pose.pose.position)

def odometryCb_tb3(msg):     # Callback Function for position tb_3

    global curr_odom_tb3
    curr_odom_tb3 = msg
    #print("Tb3",  msg.pose.pose.Position)
    # print("Tb 3 Postiion:\n", curr_odom_tb3.pose.pose.position)

# def odometryCb_tb1_tr(msg):     # Callback Function for position tb_1
#
#     global curr_odom_tb1_tr
#     curr_odom_tb1_tr = msg
#     #print("Tb1", msg.pose.pose.position)
#     # print("Tb 1 TR Position:\n", curr_odom_tb1_tr.pose.pose.position)
#
# def odometryCb_tb2_tr(msg):     # Callback Function for position tb_2
#
#     global curr_odom_tb2_tr
#     curr_odom_tb2_tr = msg
#     #print("Tb2",  msg.pose.pose.position)
#     # print("Tb 2  TR Position:\n", curr_odom_tb2_tr.pose.pose.position)
#
# def odometryCb_tb3_tr(msg):     # Callback Function for position tb_3
#
#     global curr_odom_tb3_tr
#     curr_odom_tb3_Tr = msg
#     #print("Tb3",  msg.pose.pose.Position)
#     # print("Tb 3 TR Postiion:\n", curr_odom_tb3_tr.pose.pose.position)


if __name__ == "__main__":

    rospy.init_node('relative_odom', anonymous=True)     # Make Node
    rate = rospy.Rate(1)     # Frequency in Hz

    rospy.Subscriber('/tb3_0/odom',Odometry,odometryCb_tb1)     # Subscribe to Odometry tb_1
    rospy.Subscriber('/tb3_1/odom',Odometry,odometryCb_tb2)     # Subscribe to Odometry tb_2
    rospy.Subscriber('/tb3_2/odom',Odometry,odometryCb_tb3)     # Subscribe to Odometry tb_3
    # rospy.Subscriber('/tb3_0/truth',Odometry,odometryCb_tb1_tr)     # Subscribe to Truth Odometry tb_1
    # rospy.Subscriber('/tb3_1/truth',Odometry,odometryCb_tb2_tr)     # Subscribe to Truth Odometry tb_2
    # rospy.Subscriber('/tb3_2/truth',Odometry,odometryCb_tb3_tr)     # Subscribe to Truth Odometry tb_3

    while (not rospy.is_shutdown()):

        rate.sleep()

        range_tb1_tb2 = Range()
        range_tb1_tb2.range = math.sqrt((curr_odom_tb1.pose.pose.position.x-curr_odom_tb2.pose.pose.position.x)**2+(curr_odom_tb1.pose.pose.position.y-curr_odom_tb2.pose.pose.position.y)**2)
        print("************")
        print("Distance from Turtlebot 1 to Turtlebot 2:", range_tb1_tb2.range)
        pub1 = rospy.Publisher('range_tb1_tb2', Range, queue_size=1)     # Publish Topic with Distance tb1_tb2
        # rospy.loginfo(range_tb1_tb2)
        pub1.publish(range_tb1_tb2)

        # range_tb1_tb2_tr = Range()
        # range_tb1_tb2_tr.range = math.sqrt((curr_odom_tb1_tr.pose.pose.position.x-curr_odom_tb2_tr.pose.pose.position.x)**2+(curr_odom_tb1_tr.pose.pose.position.y-curr_odom_tb2_tr.pose.pose.position.y)**2)

        # print("Real Distance from Turtlebot 1 to Turtlebot 2:", range_tb1_tb2_tr.range)
        # print("Tb1 Pos x:", range_tb1_tb2_tr.range)
        # pub4 = rospy.Publisher('range_tb1_tb2_tr', Range, queue_size=1)     # Publish Topic with Distance tb1_tb2
        # # rospy.loginfo(range_tb1_tb2_tr)
        # pub4.publish(range_tb1_tb2_tr)

        # print("----------")
        range_tb1_tb3 = Range()
        range_tb1_tb3.range = math.sqrt((curr_odom_tb1.pose.pose.position.x-curr_odom_tb3.pose.pose.position.x)**2+(curr_odom_tb1.pose.pose.position.y-curr_odom_tb3.pose.pose.position.y)**2)
        print("Distance from Turtlebot 1 to Turtlebot 3:", range_tb1_tb3.range)
        pub2 = rospy.Publisher('range_tb1_tb3', Range, queue_size=1)     # Publish Topic with Distance tb1_tb3
        # rospy.loginfo(range_tb1_tb3)
        pub2.publish(range_tb1_tb3)

        # range_tb1_tb3_tr = Range()
        # range_tb1_tb3_tr.range = math.sqrt((curr_odom_tb1_tr.pose.pose.position.x-curr_odom_tb3_tr.pose.pose.position.x)**2+(curr_odom_tb1_tr.pose.pose.position.y-curr_odom_tb3_tr.pose.pose.position.y)**2)
        # print("Real Distance from Turtlebot 1 to Turtlebot 3:", range_tb1_tb3_tr.range)
        # pub5 = rospy.Publisher('range_tb1_tb3_tr', Range, queue_size=1)     # Publish Topic with Distance tb1_tb3
        # # rospy.loginfo(range_tb1_tb3_tr)
        # pub5.publish(range_tb1_tb3_tr)

        # print("----------")
        range_tb2_tb3 = Range()
        range_tb2_tb3.range = math.sqrt((curr_odom_tb2.pose.pose.position.x-curr_odom_tb3.pose.pose.position.x)**2+(curr_odom_tb2.pose.pose.position.y-curr_odom_tb3.pose.pose.position.y)**2)
        print("Distance from Turtlebot 2 to Turtlebot 3:", range_tb2_tb3.range)
        pub3 = rospy.Publisher('range_tb2_tb3', Range, queue_size=1)     # Publish Topic with Distance tb2_tb3
        # rospy.loginfo(range_tb2_tb3)
        pub3.publish(range_tb2_tb3)

        # range_tb2_tb3_tr = Range()
        # range_tb2_tb3_tr.range = math.sqrt((curr_odom_tb2_tr.pose.pose.position.x-curr_odom_tb3_tr.pose.pose.position.x)**2+(curr_odom_tb2_tr.pose.pose.position.y-curr_odom_tb3_tr.pose.pose.position.y)**2)
        # print("Real Distance from Turtlebot 2 to Turtlebot 3:", range_tb2_tb3_tr.range)
        # pub6 = rospy.Publisher('range_tb2_tb3_tr', Range, queue_size=1)     # Publish Topic with Distance tb2_tb3
        # # rospy.loginfo(range_tb2_tb3_tr)
        # pub6.publish(range_tb2_tb3_tr)

    rospy.spin()
