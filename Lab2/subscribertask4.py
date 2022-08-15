#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt
import time


# PID variable

error_angle = 0
error_linear = 0

sumError_angle = 0
sumError_linear = 0

rateError_linear = 0
rateError_angle = 0

currentTime = time.time()
previousTime = time.time()

lastError_linear = 0
lastError_angle = 0


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber =  rospy.Subscriber("/odom", Odometry, self.update_pose)

        self.pose = Odometry()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
        self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)

        self.pose.pose.pose.orientation.x = round(self.pose.pose.pose.orientation.x, 4)
        self.pose.pose.pose.orientation.y = round(self.pose.pose.pose.orientation.y, 4)
        self.pose.pose.pose.orientation.z = round(self.pose.pose.pose.orientation.z, 4)
        self.pose.pose.pose.orientation.w = round(self.pose.pose.pose.orientation.w, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y, goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""

        orientation_list = [self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y,self.pose.pose.pose.orientation.z,self.pose.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        rospy.loginfo('roll %d pitch %d yaw %d', roll, pitch, yaw)


        return (self.steering_angle(goal_pose) - yaw)

    def move2goal(self):

        global currentTime, previousTime, error_linear, sumError_linear, rateError_linear, lastError_linear, error_angle, sumError_angle, rateError_angle, lastError_angle
        """Moves the turtle to the goal."""
        goal_pose = Odometry()

        # Get the input from the user.
        goal_pose.pose.pose.position.x = float(input("Set your x goal: "))
        goal_pose.pose.pose.position.y = float(input("Set your y goal: "))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = input("Set your tolerance: ")

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
    
            currentTime = time.time()
            elapsedTime = (currentTime - previousTime)
            error_linear = self.linear_vel(goal_pose)

            error_angle = self.angular_vel(goal_pose)

            sumError_linear = sumError_linear + (error_linear * elapsedTime)
            rateError_linear = (error_linear - lastError_linear)/elapsedTime

            sumError_angle = sumError_angle + (error_angle * elapsedTime)
            rateError_angle = (error_angle - lastError_angle)/elapsedTime


            vel_pid = (0.15 * error_linear) + (0 * sumError_linear) + (0 * rateError_linear)
            
            ang_pid = (0.1 * error_angle) + (0 * sumError_angle) + (0 * rateError_angle)

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = (vel_pid)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = ang_pid 

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)


            previousTime = currentTime
            lastError_angle = error_angle
            lastError_linear = error_linear

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        rospy.loginfo("reached")
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
