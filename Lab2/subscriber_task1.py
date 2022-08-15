#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import LaserScan

value =0

def callback(data):
    y = []
    global value
    rospy.loginfo(data)
    for x in data.ranges:
	if x != float('inf'):
            y.append(x)
    value = (min(y))
    #rospy.loginfo(value)

def listener():
    rospy.init_node('turtlebot_controller', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
