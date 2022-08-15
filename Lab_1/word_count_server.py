#!/usr/bin/env python



from __future__ import print_function

import rospy
from alam_lab1.srv import WordCount, WordCountResponse


def count_words(req):
    return WordCountResponse(len(req.words.split()))

def count():
    rospy.init_node('word_count_server')
    s = rospy.Service('word_count', WordCount, count_words)
    print("Ready to count word")
    rospy.spin()

if __name__ == "__main__":
    count()
