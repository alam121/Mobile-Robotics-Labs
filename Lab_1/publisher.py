#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
rospy.init_node('topic_publisher')
pub = rospy.Publisher('counter', Int32,queue_size=10)
rate = rospy.Rate(2)
count = 4
while not rospy.is_shutdown():
 count= count + count
 pub.publish(count)
 rospy.loginfo(count)
 rate.sleep()

