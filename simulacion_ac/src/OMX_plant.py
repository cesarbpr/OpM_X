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
    #   n=353.5; g=9.81;
    n=350.5
    g=9.81
    Jm=0.0000071
    Jg=0.0000053
    #   Jeq=(n**2)*Jm+Jg;Beq=(n**2)*Bm+Bg;
    Jeq=0.0037
    Beq=0.0012

    # DATOS DEL SUB-SISTEMA ELECTRICO
    #   Km=0.0458; Kb=0.0458; KA=8.5; Ra=2.49;Ka=8.5;
    Km=0.005
    Kb=1
    KA=1
    Ra=1
    Ka=1
    T = 0.02

    def __init__(self):
        pub_topic_q_real="q_real"
        sub_topic_u="u"

        self.pub_qreal=rospy.Publisher(pub_topic_q_real,Float64MultiArray,queue_size=10)
        self.u_suscriber=rospy.Subscriber(sub_topic_u,Float64MultiArray,self.cb_modelo)

    def cb_qreal_in(self,q_real):
        self.q=np.transpose(q_real.data)

    def publish_present_u(self):

        pub_array = Float64MultiArray()
        pub_array.data[0] = self.u[0,0]
        pub_array.data[1] = self.u[1,0]
        self.pub_u.publish(pub_array)
    
    def cb_modelo(self, u):
        # MODELO DINAMICO DEL SISTEMA 
        M11 = U_lin*(Iyy3 + Iyy4 + Jeq + m4*(L3_y**2 + 2*np.cos(q[1,0])*L3_y*l4_y + l4_y**2) + l3_y**2*m3);
        M12 = U_lin*(Iyy4 + m4*(l4_y**2 + L3_y*np.cos(q[1,0])*l4_y));
        M21 = M12;
        M22 = U_lin*(m4*l4_y**2 + Iyy4 + Jeq);
        M   = np.array([[M11, M12],
                    [M21, M22]]);
        #######
        P11 = U_lin*(Beq - L3_y*l4_y*m4*dq[1,0]*np.sin(q[1,0]));
        P12 = U_lin*(-L3_y*l4_y*m4*np.sin(q[1,0])*(dq[0,0] + dq[1,0]));
        P21 = U_lin*(L3_y*l4_y*m4*dq[0,0]*np.sin(q[1,0]));
        P22 = U_lin*(Beq);

        P   = np.array([[P11, P12],
                [P21, P22]])

        #####
        D11 = U_lin*(-g*m4*(l4_y*np.cos(q[0,0] + q[1,0]) + L3_y*np.cos(q[0,0])) - g*l3_y*m3*np.cos(q[0,0]));
        D21 = U_lin*(-g*l4_y*m4*np.cos(q[0,0] + q[1,0]));
        d = np.array([[D11], [D21]])
        dq = dq + T * np.linalg.solve(M, u - np.dot(P, dq) - d)
        q = q + T * dq


def main():
    rospy.init_node('adaptative_controller')
    rospy.Subscriber('q_des',Float64MultiArray,function)
    pub = rospy.Publisher('q_ref',Float64MultiArray,queue_size=1)
    freq = 10
    rate = rospy.Rate(freq) #10 Hz
    control=controler()

    while not rospy.is_shutdown():
        control.publish_present_u()
        rate.sleep()

if __name__ == "__main__":
    main()