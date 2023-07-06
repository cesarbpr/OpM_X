#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

class controler:
    # VARIABLES:
    Lambda=0.5
    ld = 8
    Kd = 1
    Gamma = 0.008
    #tap=1;
    u_lim=2.2
    # PARAMETROS DE CONTROL
    # I = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
    I = np.eye(2,2)
    Ld = ld*I
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

    # Keq=Beq+(n**2)*Km*Kb/Ra;

    # CONDICIONES INICIALES
    # VALORES INICIALES
    # q = np.array([[np.deg2rad(0)], [np.deg2rad(0)]], dtype=np.float64)
    dq = np.array([[0], [0]], dtype=np.float64)
    qe= np.array([[np.deg2rad(0)], [np.deg2rad(0)]], dtype=np.float64) 
    dqe=np.array([[0], [0]], dtype=np.float64)
    dqd = np.array([[0], [0]], dtype=np.float64)
    ddqd =  np.array([[0], [0]], dtype=np.float64)
    qd=np.array([[0], [0]], dtype=np.float64)
    qtilde = np.array([[0], [0]], dtype=np.float64)
    dqr = np.array([[np.deg2rad(0)], [np.deg2rad(0)]], dtype=np.float64)
    ddqr =  np.array([[0], [0]], dtype=np.float64)
    s =  np.array([[0], [0]], dtype=np.float64)
    u =  np.array([[0], [0]], dtype=np.float64)
    dqtilde =  np.array([[0], [0]], dtype=np.float64)
    # print("dqe:\n")
    # print(dqe)
    # print("\n")
    U_lin=1/(Km*n)
    a1=m3*l3_y**2+Iyy3+Iyy4+m4*(L3_y**2+l4_y**2)
    a2=m4*L3_y*l4_y
    a3=Jeq; a4=Beq
    a5=Iyy4+m4*l4_y**2
    a6=m4*L3_y+m3*l3_y
    a7=m4*l4_y
    ae=U_lin*np.array([[a1],[a2],[a3],[a4],[a5],[a6],[a7]])
    u_pas=np.array([[0], [0]])
    T = 0.02

    def __init__(self):
        pub_topic_u="u"
        sub_topic_q_des="q_des"
        sub_topic_q_real="q_real"

        self.pub_u=rospy.Publisher(pub_topic_u,Float64MultiArray,queue_size=10)
        self.qd_suscriber=rospy.Subscriber(sub_topic_q_des,Float64MultiArray,self.cb_control)
        self.qreal_suscriber=rospy.Subscriber(sub_topic_q_real,Float64MultiArray,self.cb_qreal_in)

    def cb_qreal_in(self,q_real):
        self.q=np.transpose(q_real.data)

    def publish_present_u(self):

        pub_array = Float64MultiArray()
        pub_array.data[0] = self.u[0,0]
        pub_array.data[1] = self.u[1,0]
        self.pub_u.publish(pub_array)
    
    def cb_control(self, q_des):
        # OBSERVADOR DE VELOCIDAD
        qd = np.transpose(q_des.data)
        self.dqe = self.dqd + self.Ld@(self.q - self.qe)
        self.qe = self.qe + self.T*self.dqe
    
        # ERROR DE POSICION Y DE VELOCIDAD   
        self.qtilde = self.q - qd
        self.dqtilde = self.dqe - self.dqd

        # FUNCION DE DESLIZAMIENTO 
        self.s = self.dqtilde + self.Lambda*self.qtilde

        # VELOCIDAD Y ACELERACION DE REFERENCIA
        self.dqr = self.dqd - self.Lambda*self.qtilde
        self.ddqr = self.ddqd - self.Lambda*self.dqtilde

        # MATRIZ Y
        Y11=self.ddqr[0,0]

        Y12=np.cos(self.qe[1,0])*(2*self.ddqr[0,0]+self.ddqr[1,0])-np.sin(self.qe[1,0])*(2*self.dqr[0,0]*self.dqr[1,0]+self.dqr[1,0]**2)
        Y13=self.ddqr[0,0]
        Y14=self.dqr[0,0]
        Y15=self.ddqr[1,0]
        Y16=-self.g*np.cos(self.qe[0,0])
        Y17=-self.g*np.cos(self.qe[0,0]+self.qe[1,0])
        Y21=0
        Y22=np.cos(self.qe[1,0])*self.ddqr[0,0]+np.sin(self.qe[1,0])*self.dqr[0,0]**2
        Y23=self.ddqr[1,0]
        Y24=self.dqr[1,0]
        Y25=self.ddqr[0,0]+self.ddqr[1,0]
        Y26=0
        Y27=-self.g*np.cos(self.qe[0,0]+self.qe[1,0])

        Y = np.array([[Y11, Y12, Y13, Y14, Y15, Y16, Y17],
                    [Y21, Y22, Y23, Y24, Y25, Y26, Y27]])


        # ESTIMACION DE PARAMETROS
        Y_trans = np.transpose(Y) 

        self.ae = self.ae - self.T*self.Gamma*Y_trans@self.s

        # LEY DE CONTROL 
        self.u=Y@self.ae-self.Kd*self.s

        # Limitador de corriente:
        if self.u[0] > self.u_lim or self.u[0] < -self.u_lim:
            self.u[0] = self.u_pas[0]
        else:
            self.u_pas[0] = self.u[0]

        if self.u[1] > self.u_lim or self.u[1] < -self.u_lim:
            self.u[1] = self.u_pas[1]
        else:
            self.u_pas[1] = self.u[1]


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