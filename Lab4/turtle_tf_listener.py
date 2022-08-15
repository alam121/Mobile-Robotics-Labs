#!/usr/bin/env python  
import roslib
roslib.load_manifest('alamlab5')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener2 = tf.TransformListener()
    listener3 = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    killer = rospy.ServiceProxy('kill', turtlesim.srv.Kill)
    killer("turtle1")   
 
    spawner(4, 2, 0, 'turtle2')
    spawner(2, 5, 0, 'turtle3')
    spawner(0, 0, 0, 'turtle1')
   

    turtle_vel2 = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    turtle_vel3 = rospy.Publisher('turtle3/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans1,rot1) = listener2.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
            (trans2,rot2) = listener3.lookupTransform('/turtle3', '/carrot2', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular1 = 4 * math.atan2(trans1[1], trans1[0])
        linear1 = 0.5 * math.sqrt(trans1[0] ** 2 + trans1[1] ** 2)


        angular2 = 4 * math.atan2(trans2[1], trans2[0])
        linear2 = 0.5 * math.sqrt(trans2[0] ** 2 + trans2[1] ** 2)

        cmd1 = geometry_msgs.msg.Twist()
        cmd2 = geometry_msgs.msg.Twist()

        cmd1.linear.x = linear1
        cmd1.angular.z = angular1

        cmd2.linear.x = linear2
        cmd2.angular.z = angular2

        turtle_vel2.publish(cmd1)
        turtle_vel3.publish(cmd2)

        rate.sleep()
