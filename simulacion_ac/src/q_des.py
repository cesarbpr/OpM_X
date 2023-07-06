#!/usr/bin/env python3
import rospy
import numpy as np
#from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

if __name__ == "__main__":

    rospy.init_node("qd_publisher")
    pub=rospy.Publisher('q_des',Float64MultiArray, queue_size=1)
    rate = rospy.Rate(2) #2 Hz

    # CONDICIONES INICIALES 
    k=0
    A=0.5
    nn=100
    qd1=np.deg2rad(0)
    qd2=np.deg2rad(0)
    q_des = np.array([qd1,qd2])

    while not rospy.is_shutdown():
        var= Float64MultiArray()
        var.data = q_des
        rospy.loginfo(str(var.data))
        pub.publish(var)

            # Codigo:
        if k >= 0 and k <= nn/4:
            qd1  = 0*A
            qd2 = 0*A
        elif k >= nn/4 and k <= nn/2:
            qd1 = 0.2*A
            qd2 = 0.7*A
        elif k >= nn/2 and k <= 3*nn/4:
            qd1 = 0.8*A
            qd2 = 0.3*A
        elif k >= 3*nn/4 and k <= nn:
            qd1 = 0.3*A
            qd2 = 0.5*A
        
        q_des = np.array([qd1,qd2])
        k = k + 1
        rospy.loginfo(str(k))
        rate.sleep()

