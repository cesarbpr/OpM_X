#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

class robot:

    # DATOS DEL SUB-SISTEMA MECANICO
    # Masas
    m3=0.135
    m4=0.236
    #Longitudes
    l3_y=0.0933
    L3_y=0.124
    l4_y=0.060
    L4_y=0.126
    # Inercias
    Ixx3=1358961*1e-09
    Iyy3=1350228*1e-09
    Izz3=24835*1e-09
    Ixx4=1128335*1e-09
    Iyy4=1077791*1e-09
    Izz4=182774*1e-09
    Bm=0.0001
    Bg=0.01
    #   n=353.5; self.g=9.81;
    n=350.5
    g=9.81
    Jm=0.0000071
    Jg=0.0000053
    #   Jeq=(n**2)*Jm+Jg;self.Beq=(n**2)*Bm+Bg;
    Jeq=0.0037
    Beq=0.0012

    # DATOS DEL SUB-SISTEMA ELECTRICO
    #   Km=0.0458; Kb=0.0458; KA=8.5; Ra=2.49;Ka=8.5;
    Km=0.005
    Kb=1
    KA=1
    Ra=1
    Ka=1
    U_lin=1/(Km*n)
    T = 0.01

    # Posción y velocidad inicial OMX
    q = np.array([[np.deg2rad(0)], [np.deg2rad(0)]], dtype=np.float64)
    dq = np.array([[0], [0]], dtype=np.float64)
    q_ac=np.array([0,0], dtype=np.float64)
    u = np.array([[0], [0]], dtype=np.float64)

    def __init__(self):
        pub_topic_q_real="q_real"
        sub_topic_u="u"

        self.pub_qreal=rospy.Publisher(pub_topic_q_real,Float64MultiArray,queue_size=10)
        self.u_suscriber=rospy.Subscriber(sub_topic_u,Float64MultiArray,self.cb_u)

    def publish_present_q(self):
        pub_array = Float64MultiArray()
        pub_array.data  = [0,0]
        pub_array.data[0] = self.q[0,0]
        pub_array.data[1] = self.q[1,0]
        self.pub_qreal.publish(pub_array)
    
    def cb_u(self, u_data):
        u1=u_data.data[0]
        u2=u_data.data[1]
        self.u = np.array([[u1], [u2]])

    def OMX_modelo(self):

        # MODELO DINAMICO DEL SISTEMA 
        M11 = self.U_lin*(self.Iyy3 + self.Iyy4 + self.Jeq + self.m4*(self.L3_y**2 + 2*np.cos(self.q[1,0])*self.L3_y*self.l4_y + self.l4_y**2) + self.l3_y**2*self.m3);
        M12 = self.U_lin*(self.Iyy4 + self.m4*(self.l4_y**2 + self.L3_y*np.cos(self.q[1,0])*self.l4_y));
        M21 = M12;
        M22 = self.U_lin*(self.m4*self.l4_y**2 + self.Iyy4 + self.Jeq);
        M   = np.array([[M11, M12],[M21, M22]]);
        #######
        P11 = self.U_lin*(self.Beq - self.L3_y*self.l4_y*self.m4*self.dq[1,0]*np.sin(self.q[1,0]));
        P12 = self.U_lin*(-self.L3_y*self.l4_y*self.m4*np.sin(self.q[1,0])*(self.dq[0,0] + self.dq[1,0]));
        P21 = self.U_lin*(self.L3_y*self.l4_y*self.m4*self.dq[0,0]*np.sin(self.q[1,0]));
        P22 = self.U_lin*(self.Beq);

        P   = np.array([[P11, P12],
                [P21, P22]])

        #####
        D11 = self.U_lin*(-self.g*self.m4*(self.l4_y*np.cos(self.q[0,0] + self.q[1,0]) + self.L3_y*np.cos(self.q[0,0])) - self.g*self.l3_y*self.m3*np.cos(self.q[0,0]));
        D21 = self.U_lin*(-self.g*self.l4_y*self.m4*np.cos(self.q[0,0] + self.q[1,0]));
        d = np.array([[D11], [D21]])
        self.dq = self.dq + self.T * np.linalg.solve(M, self.u - np.dot(P, self.dq) - d)
        self.q = self.q + self.T * self.dq
        self.q_ac=([self.q[0,0],self.q[1,0]])


def main():
    rospy.init_node('OMX_plant')
    freq = 100
    rate = rospy.Rate(freq) #10 Hz
    OMX=robot()

    while not rospy.is_shutdown():
        OMX.OMX_modelo()
        OMX.publish_present_q()
        q_re=np.round(np.rad2deg(OMX.q_ac),2)
        #print("q1:",OMX.q[0],"\n")
        #print("\n ------- \n")
        #print("q2:",OMX.q[1],"\n")
        rospy.loginfo(str(q_re))
        rate.sleep()

if __name__ == "__main__":
    main()