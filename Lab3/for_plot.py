#!/usr/bin/python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import math
import matplotlib.pyplot as plt

# Global variables
pub_handle_py = rospy.Publisher("sensor_for_box", Range, queue_size=10)
dataToPublish = Range()
afUltrasoundSamples = list()
afUltrasoundSamples_front = list()
afUltrasoundSamples_back = list()
afUltrasoundSamples_right = list()
afUltrasoundSamples_left = list()

calculatedMean_front = 0
calculatedMean_back = 0
calculatedMean_left = 0
calculatedMean_right = 0


def call_backfront(recvd_msg):
    global afUltrasoundSamples_front
    global afUltrasoundSamples
    global calculatedMean_front

    afUltrasoundSamples.append(recvd_msg.range)
    calculatedMean_front = calcAvg(afUltrasoundSamples)
    calculatedVariance = calcVariance(afUltrasoundSamples)
    calculatedStdDeviation = calcStandardDeviation(afUltrasoundSamples)

    rospy.loginfo("for front Mean is %f variance is %f Standard Deviation is : %f"% (calculatedMean_front, calculatedVariance,
                calculatedStdDeviation,
            )
        )
    
    afUltrasoundSamples_front.append(calculatedMean_front)


def call_backback(recvd_msg):
    global afUltrasoundSamples_back
    global afUltrasoundSamples
    global calculatedMean_back

    afUltrasoundSamples.append(recvd_msg.range)

    calculatedMean_back = calcAvg(afUltrasoundSamples)
    calculatedVariance = calcVariance(afUltrasoundSamples)
    calculatedStdDeviation = calcStandardDeviation(afUltrasoundSamples)

    rospy.loginfo("for back Mean is %f variance is %f Standard Deviation is : %f" %(calculatedMean_back, calculatedVariance, calculatedStdDeviation))

 
    afUltrasoundSamples_back.append(calculatedMean_back)



def call_backright(recvd_msg):
    global afUltrasoundSamples_right
    global afUltrasoundSamples
    global calculatedMean_right

    afUltrasoundSamples.append(recvd_msg.range)
    calculatedMean_right = calcAvg(afUltrasoundSamples)
    calculatedVariance = calcVariance(afUltrasoundSamples)
    calculatedStdDeviation = calcStandardDeviation(afUltrasoundSamples)

    rospy.loginfo(
            "for right Mean is %f variance is %f Standard Deviation is : %f"
            % (
                calculatedMean_right,
                calculatedVariance,
                calculatedStdDeviation,
            )
        )

    afUltrasoundSamples_right.append(calculatedMean_right)



def call_backleft(recvd_msg):
    global afUltrasoundSamples_left
    global afUltrasoundSamples
    global calculatedMean_left


    afUltrasoundSamples.append(recvd_msg.range)
    calculatedMean_left = calcAvg(afUltrasoundSamples)
    calculatedVariance = calcVariance(afUltrasoundSamples)
    calculatedStdDeviation = calcStandardDeviation(afUltrasoundSamples)

    rospy.loginfo(
            "for left Mean is %f variance is %f Standard Deviation is : %f"
            % (
                calculatedMean_left,
                calculatedVariance,
                calculatedStdDeviation,
            )
        )

    afUltrasoundSamples_left.append(calculatedMean_left)



def calcAvg(data):
    return sum(data) / len(data)


def calcVariance(data, ddof=0):
    n = len(data)
    mean = sum(data) / n

    variance = sum((x - mean) ** 2 for x in data) / (n - ddof)

    return variance


def calcStandardDeviation(data):
    var = calcVariance(data)
    std_dev = math.sqrt(var)
    return std_dev


def main():
    rospy.init_node("sensor")
    rospy.Subscriber("/ultrasound_front", Range, call_backfront, queue_size=10)
    rospy.Subscriber("/ultrasound_back", Range, call_backback, queue_size=10)
    rospy.Subscriber("/ultrasound_right", Range, call_backright, queue_size=10)
    rospy.Subscriber("/ultrasound_left", Range, call_backleft, queue_size=10)

    print(afUltrasoundSamples_left)
    print(afUltrasoundSamples_right)
    print(afUltrasoundSamples_front)
    print(afUltrasoundSamples_back)
    


    rospy.spin()


if __name__ == "__main__":
    try:
        main()
        plt.axhline(y=calculatedMean_back, color='r', linestyle='-')
        plt.axhline(y=calculatedMean_front, color='r', linestyle='-')
        plt.axvline(x=calculatedMean_right, color='r', linestyle='-')
        plt.axvline(x=calculatedMean_left, color='r', linestyle='-')
        plt.show()
    except rospy.ROSInterruptException:
        pass
