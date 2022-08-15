#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

value =0
value1=0

def callback(data):
    rate = rospy.Rate(10)
    y = []

    w = []
    global value
    for x in data.ranges:
	if x != float('inf'):
            y.append(x)
    value = (min(y))
    min_angle=data.angle_min
    max_angle=data.angle_max

    rospy.loginfo("minimum value in array is %f", value)
    bearing=min_angle+ data.ranges.index(value) * max_angle/(len(data.ranges))
    rospy.loginfo("corrosponding angle is %f", bearing)

    y1 =[]


    value1 = (max(y))
    rospy.loginfo("maximum value in array is %f", value1)
    bearing1=min_angle+data.ranges.index(value1) * max_angle/(len(data.ranges))
    rospy.loginfo("corrosponding angle is %f", bearing1)
    
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    


    rospy.loginfo("0 value is at %f", data.ranges[0])
    w.append(data.ranges[0])
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = -1
    velocity_publisher.publish(vel_msg)
    

    rospy.loginfo(w)
    while round(data.ranges[0],1) == round(value,1):
        rospy.loginfo("0 value is at %f", data.ranges[0])
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

        
        rate.sleep()





        
      
    #rospy.loginfo(value)

def listener():
    rospy.init_node('turtlebot_controller', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
