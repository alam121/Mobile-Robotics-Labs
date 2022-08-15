#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

global check 
check = 0;
def callback(msg):
	check = check + msg.data
	print check		

rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('counter', Int32, callback)
rospy.spin()

