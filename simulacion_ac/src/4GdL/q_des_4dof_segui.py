#!/usr/bin/env python3
import rospy
import numpy as np
#from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

if __name__ == "__main__":

    rospy.init_node("qd_publisher")
    pub=rospy.Publisher('q_des',Float64MultiArray, queue_size=20)
    freq = 100
    rate = rospy.Rate(freq) #2 Hz

    # CONDICIONES INICIALES 
    tipo = "referencia" #"seguimiento"
    k=0
    A=0.5
    As=0.5
    W=0.008
    B=0
    nn=1000
    qd1=np.deg2rad(0)
    qd2=np.deg2rad(0)
    qd3=np.deg2rad(0)
    qd4=np.deg2rad(0)
    q_des = np.array([qd1,qd2,qd3,qd4,0])

    while not rospy.is_shutdown():
        var= Float64MultiArray()
        var.data = q_des
        q_deg=np.round(np.rad2deg(q_des),2)
        rospy.loginfo(str(q_deg))
        pub.publish(var)
        
        if tipo == "referencia":
            
            if k >= 0 and k <= nn/4:
                qd1 = np.deg2rad(0)
                qd2 = np.deg2rad(0)
                qd3 = np.deg2rad(0)
                qd4 = np.deg2rad(0)
            elif k >= nn/4 and k <= nn/2:
                qd1 = np.deg2rad(20)
                qd2 = np.deg2rad(20)
                qd3 = np.deg2rad(15)
                qd4 = np.deg2rad(20)
            elif k >= nn/2 and k <= 3*nn/4:
                qd1 = np.deg2rad(30)
                qd2 = np.deg2rad(-10)
                qd3 = np.deg2rad(25)
                qd4 = np.deg2rad(30)
            elif k >= 3*nn/4 and k <= nn:
                qd1 = np.deg2rad(0)
                qd2 = np.deg2rad(0)
                qd3 = np.deg2rad(0)
                qd4 = np.deg2rad(0)
            else:
                qd1 = np.deg2rad(0)
                qd2 = np.deg2rad(0)
                qd3 = np.deg2rad(0)
                qd4 = np.deg2rad(0)
                k = 0
        elif tipo == "seguimiento":
            qd1 = (np.sin(W*k)*As+B*k)
            qd2 = (np.cos(W*k)*0.2+0.1)
            qd3 = -np.sin(W*k)*As-B*k
            qd4 = -np.cos(W*k)*As-B*k           
        else:
            qd1  = 0
            qd2  = 0
            qd3  = 0
            qd4  = 0


        #qd1  = 0
        #qd2  = 0
        #qd3  = 0
        #qd4  = 0
        
        q_des = np.array([qd1,qd2,qd3,qd4,0])
        k = k + 1
        #rospy.loginfo(str(k))
        rate.sleep()

