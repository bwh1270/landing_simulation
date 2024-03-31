#! /usr/bin/env python
import rospy
from std_msgs.msg import Float32


def wind_generator():

    rospy.init_node("wind_generator_test_node", anonymous=True)
    pub = rospy.Publisher("/test", Float32, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():

        windMsg = Float32()
        windMsg.data = 1.0

        pub.publish(windMsg)
        rate.sleep()


if __name__ == "__main__":
    try:
        wind_generator()
    except rospy.ROSInterruptException:
        pass