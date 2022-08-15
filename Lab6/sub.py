#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped as pwc
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
import numpy as np
from numpy import random 

arr_odom_x =[]
arr_odom_y =[]
arr_odom_theta =[]


arr_odom_filt_x =[]
arr_odom_filt_y =[]
arr_odom_filt_theta =[]


arr_odom_ekf_x =[]
arr_odom_ekf_y =[]
arr_odom_ekf_theta =[]

def odometryCb(msg):
    global arr_odom_x, arr_odom_y,arr_odom_theta
    
    arr_odom_x.append(msg.pose.pose.position.x)
    arr_odom_y.append(msg.pose.pose.position.y)
    
    orientation_list = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list) 
    
    arr_odom_theta.append(yaw)
    #print ('simple odom wala run horha hai')
    #print(arr_odom_theta)

    #if len(arr_odom_x) > 100:
     #   plt.figure(1)
    	#plt.plot(arr_odom_x,arr_odom_y,'r^')
    	#plt.axis('equal')
        #plt.show()
    	#plt.pause(0.05)

def odometry_husk(msg):
    global arr_odom_filt_x, arr_odom_filt_y,arr_odom_filt_theta
    
    arr_odom_filt_x.append(msg.pose.pose.position.x)
    arr_odom_filt_y.append(msg.pose.pose.position.y)
    
    orientation_list = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list) 
    
    arr_odom_filt_theta.append(yaw)
    #print ('husky ka filtgered odom wala run horha hai')
    #print(arr_odom_filt_theta)

    #if len(arr_odom_x) > 100:
     #   plt.figure(2)
    	#plt.plot(arr_odom_filt_x,arr_odom_filt_y,'yo')
    	#plt.axis('equal')
    	#plt.pause(0.05)
        #plt.show()


def odometry_ekf(msg):

    global arr_odom_ekf_x, arr_odom_ekf_y,arr_odom_ekf_theta
    
    arr_odom_ekf_x.append(msg.pose.pose.position.x)
    arr_odom_ekf_y.append(msg.pose.pose.position.y)
    #print(msg.pose.pose.position)
    orientation_list = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list) 
    
    arr_odom_ekf_theta.append(yaw)
    #print ('ekf ka filttered odom wala run horha hai')
    print(len(arr_odom_ekf_x))
    main_mhr()


def main_alam():
    #plt.plot(arr_odom_x,arr_odom_y,'r^')
    #plt.plot(arr_odom_filt_x,arr_odom_filt_y,'g*')
    #plt.plot(arr_odom_ekf_x, arr_odom_ekf_y,'ro')
    #x_ang = random.randint(max(arr_odom_ekf_theta), size=(len(arr_odom_ekf_theta)))
 

    #plt.show()
    #plt.axis('equal')
    #plt.pause(10)
    x = 2
    if len(arr_odom_ekf_x) > 20:
        plt.figure(1)
    	plt.plot(arr_odom_x,arr_odom_y,'r^')

        plt.figure(2)
    	plt.plot(arr_odom_filt_x,arr_odom_filt_y,'yo')

        plt.figure(3)
        plt.plot(arr_odom_ekf_x, arr_odom_ekf_y,'b-')

    	plt.axis('equal')
    	plt.show()
    	plt.pause(0.05)
	
 
   
 



if __name__ == "__main__":
    rospy.init_node('oodometry', anonymous=True) #make node 
    rospy.Subscriber('/odom',Odometry,odometryCb)
    rospy.Subscriber('/odometry/filtered',Odometry,odometry_husk)
    rospy.Subscriber('/robot_pose_ekf/odom_combined',pwc,odometry_ekf)
    main_alam()
    rospy.spin()
