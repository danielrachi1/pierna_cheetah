#!/usr/bin/env python3
import rospy

if __name__ =='__main__':
    rospy.init_node("test_node")

    rospy.loginfo("Hello from test node")
    
    rate= rospy.Rate(10)

    #Checks if ROS hasnt received any shutdowm notification
    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        rate.sleep()