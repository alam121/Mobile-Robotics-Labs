#!/usr/bin/env python



from __future__ import print_function

import rospy
from alam_lab1.srv import WordCount
import sys

rospy.init_node('service_client')
rospy.wait_for_service('word_count')
word_counter = rospy.ServiceProxy('word_count', WordCount)
words = ' '.join(sys.argv[1:])
word_count = word_counter(words)


print (words, '->',word_count.count)



