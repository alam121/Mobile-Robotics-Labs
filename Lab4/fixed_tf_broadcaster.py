#!/usr/bin/env python  
import roslib
roslib.load_manifest('alamlab5')
from tf.transformations import quaternion_from_euler
import rospy
import tf
import numpy

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br1 = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(30.0))

    while not rospy.is_shutdown():
        br1.sendTransform((0.0, 2.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "carrot1",
                         "turtle1")

        br2.sendTransform((0.0, 1.0, 0.0),
                         (q[0], q[1], q[2], q[3]),
                         rospy.Time.now(),
                         "carrot2",
                         "turtle2")
        rate.sleep()
