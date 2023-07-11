#!/usr/bin/env python3
import rospy
import numpy as np
#from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

if __name__ == "__main__":

    rospy.init_node("qd_publisher")
    pub=rospy.Publisher('q_des',Float64MultiArray, queue_size=20)
    freq = 10
    rate = rospy.Rate(freq) #2 Hz

    # CONDICIONES INICIALES 
    k=0
    A=0.5
    nn=200
    qd1=np.deg2rad(0)
    qd2=np.deg2rad(0)
    qd3=np.deg2rad(0)
    qd4=np.deg2rad(0)
    q_des = np.array([qd1,qd2,qd3,qd4])

    while not rospy.is_shutdown():
        var= Float64MultiArray()
        var.data = q_des
        q_deg=np.round(np.rad2deg(q_des),2)
        rospy.loginfo(str(q_deg))
        pub.publish(var)
        
            # Codigo:
        if k >= 0 and k <= nn/4:
            qd1 = np.deg2rad(0)
            qd2 = np.deg2rad(0)
            qd3 = np.deg2rad(0)
            qd4 = np.deg2rad(0)
        elif k >= nn/4 and k <= nn/2:
            qd1 = np.deg2rad(30)
            qd2 = np.deg2rad(-30)
            qd3 = np.deg2rad(50)
            qd4 = np.deg2rad(-50)
        elif k >= nn/2 and k <= 3*nn/4:
            qd1 = np.deg2rad(-20)
            qd2 = np.deg2rad(30)
            qd3 = np.deg2rad(-30)
            qd4 = np.deg2rad(40)
        elif k >= 3*nn/4 and k <= nn:
            qd1 = np.deg2rad(0)
            qd2 = np.deg2rad(-20)
            qd3 = np.deg2rad(0)
            qd4 = np.deg2rad(-70)
        else:
            qd1 = np.deg2rad(0)
            qd2 = np.deg2rad(0)
            qd3 = np.deg2rad(0)
            qd4 = np.deg2rad(0)
            k = 0
        
        #qd1  = 0
        #qd2  = 0
        q_des = np.array([qd1,qd2,qd3,qd4])
        k = k + 1
        #rospy.loginfo(str(k))
        rate.sleep()

