#!/usr/bin/env python3
import rospy
import numpy as np
#from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

if __name__ == "__main__":

    rospy.init_node("qd_publisher")
    pub=rospy.Publisher('q_des',Float64MultiArray, queue_size=1)
    rate = rospy.Rate(2) #2 Hz

    count = 0
    qd1=np.deg2rad(0)
    qd2=np.deg2rad(0)
    q_des = np.array([qd1,qd2])

    while not rospy.is_shutdown():
        var= Float64MultiArray()
        var.data = q_des
        rospy.loginfo(str(var.data))
        pub.publish(var)
        qd1 = qd1 + 0.01
        qd2 = qd1 + 0.01
        q_des = np.array([qd1,qd2])
        rate.sleep()