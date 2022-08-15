#!/usr/bin/python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
import math

# Global variables

data = list()
def Publisher(data_checl):
    global data
    data.append(data_check.range)
    if len(data) == 100:
        Mean = sum(data)/len(data)
        Variance = variance(data)
        Std = calcStandardDeviation(data)

        rospy.loginfo("Mean is %f variance is %f Standard Deviation is : %f"% (calculatedMean, calculatedVariance, calculatedStdDeviation,))

def variance(data):
    n = len(data)
    mean = sum(data) / n
    variance = sum((x - mean) ** 2 for x in data) / (n - 0)
    return variance

def std(data):
    var = variance(data)
    std = math.sqrt(var)
    return std


def main():
    rospy.init_node("sensor")
    sub_handle_py = rospy.Subscriber(
        "/ultrasound", Range, cbDataFromPublisher, queue_size=10
    )
    rospy.Publisher("sensor node", Range, queue_size=10)
    data = Range()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
