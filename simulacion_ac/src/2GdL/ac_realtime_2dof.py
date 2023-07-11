#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

class controler:
    # VARIABLES:
    '''
    Lambda=2
    ld = 30
    Kd = 0.1  #este desestabiliza
    Gamma = 0.005
    #tap=1;s
    u_lim=0.45
    s_lim=0.5
    '''
    Lambda=4
    ld = 40
    Kd = 0.09     #este desestabiliza
    Gamma = 0.008
    #tap=1;s
    u_lim=0.45
    s_lim=0.01
    # PARAMETROS DE CONTROL
    # I = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
    I = np.eye(2,2)
    Ld = ld*I
    # DATOS DEL SUB-SISTEMA MECANICO
    # Masas
    m3=0.200
    m4=0.235
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
    #   Jeq=(n**2)*Jm+Jg;Beq=(n**2)*Bm+Bg;
    Jeq=0.0037
    Beq=0.0012

    # DATOS DEL SUB-SISTEMA ELECTRICO
    #   Km=0.005; Kb=0.0458; KA=8.5; Ra=2.49;Ka=8.5;
    Km=0.005
    Kb=1
    KA=1
    Ra=1
    Ka=1

    # Keq=Beq+(n**2)*Km*Kb/Ra;

    # CONDICIONES INICIALES
    # VALORES INICIALES
    # q = np.array([[np.deg2rad(0)], [np.deg2rad(0)]], dtype=np.float64)
    q = np.array([[0], [0]], dtype=np.float64)
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
    # print("self.dqe:\n")
    # print(self.dqe)
    # print("\n")
    U_lin=1#/(Km*n)
    a1=m3*l3_y**2+Iyy3+Iyy4+m4*(L3_y**2+l4_y**2)
    a2=m4*L3_y*l4_y
    a3=Jeq; a4=Beq
    a5=Iyy4+m4*l4_y**2
    a6=m4*L3_y+m3*l3_y
    a7=m4*l4_y
    ae=U_lin*np.array([[a1],[a2],[a3],[a4],[a5],[a6],[a7]])
    u_pas=np.array([[0], [0]], dtype=np.float64)
    # T = 0.01

    def __init__(self,sample_time):
        #pub_topic_u="u"
        #sub_topic_q_des="q_des"
        #sub_topic_q_real="q_real"
        pub_topic_u="u"
        sub_topic_q_des="q_des"
        sub_topic_q_real="pos_present_value"  

        #Timepo de muestreo
        self.T=1/sample_time

        self.pub_u=rospy.Publisher(pub_topic_u,Float64MultiArray,queue_size=1)
        self.qd_suscriber=rospy.Subscriber(sub_topic_q_des,Float64MultiArray,self.cb_qdes_in)
        self.qreal_suscriber=rospy.Subscriber(sub_topic_q_real,Float64MultiArray,self.cb_qreal_in)
    
    # Para la simplemenación
    def cb_qreal_in(self,q_real):
        q_des1=q_real.data[2]
        q_des2=q_real.data[3]
        self.q = np.array([[q_des1], [q_des2]])

    def publish_present_u(self):

        pub_array = Float64MultiArray()
        pub_array.data = [0,0,0,0,0]
        pub_array.data[2] = self.u[0,0]
        pub_array.data[3] = self.u[1,0]
        self.pub_u.publish(pub_array)
    """

    # Para la simulación
    def cb_qreal_in(self,q_real):
        q_des1=q_real.data[0]
        q_des2=q_real.data[1]
        self.q = np.array([[q_des1], [q_des2]])

    def publish_present_u(self):
        pub_array = Float64MultiArray()
        pub_array.data = [0,0]
        pub_array.data[0] = self.u[0,0]
        pub_array.data[1] = self.u[1,0]
        self.pub_u.publish(pub_array)

    """

    def cb_qdes_in(self,q_des):
        q_des1=q_des.data[0]
        q_des2=q_des.data[1]
        self.qd = np.array([[q_des1], [q_des2]])



    def cb_control(self):
        
        # OBSERVADOR DE VELOCIDAD
        self.dqe = self.dqd + self.Ld@(self.q - self.qe)
        self.qe = self.qe + self.T*self.dqe
    
        # ERROR DE POSICION Y DE VELOCIDAD   
        self.qtilde = self.q - self.qd
        self.dqtilde = self.dqe - self.dqd

        # VELOCIDAD Y ACELERACION DE REFERENCIA
        self.dqr = self.dqd - self.Lambda*self.qtilde
        self.ddqr = self.ddqd - self.Lambda*self.dqtilde

        # MATRIZ Y

        Y11= - self.g*np.cos(self.qe[0,0])
        Y12= self.ddqr[0,0] 
        Y13= (self.L3_y**2*self.ddqr[0,0]- self.L3_y*self.g*np.cos(self.qe[0,0])) 
        Y14= (2*self.L3_y*self.ddqr[0,0]*np.cos(self.qe[1,0]) + self.L3_y*self.ddqr[1,0]*np.cos(self.qe[1,0]) - self.L3_y*self.dqe[1,0]*self.dqr[1,0]*np.sin(self.qe[1,0]) - self.L3_y*self.dqe[1,0]*self.dqr[0,0]*np.sin(self.qe[1,0]) - self.L3_y*self.dqe[0,0]*self.dqr[1,0]*np.sin(self.qe[1,0]) - self.g*np.cos(self.qe[0,0])*np.cos(self.qe[1,0]) + self.g*np.sin(self.qe[0,0])*np.sin(self.qe[1,0]))
        Y15= (self.ddqr[0,0] + self.ddqr[1,0]) 
        Y16= self.ddqr[0,0] 
        Y17= self.ddqr[0,0] 
        Y18= self.dqr[0,0]
        Y24= (-self.g*np.cos(self.qe[0,0])*np.cos(self.qe[1,0]) + self.g*np.sin(self.qe[0,0])*np.sin(self.qe[1,0]) + self.L3_y*self.ddqr[0,0]*np.cos(self.qe[1,0]) + self.L3_y*self.dqe[0,0]*self.dqr[0,0]*np.sin(self.qe[1,0]))
        Y25= (self.ddqr[0,0] + self.ddqr[1,0]) 
        Y27= self.ddqr[1,0] 
        Y28= self.dqr[1,0]
        Y = np.array([[Y11, Y12, Y13, Y14, Y15, Y16, Y17],
                      [0  ,   0,   0, Y24, Y25,  0 , Y27]])


        # ESTIMACION DE PARAMETROS
        Y_trans = np.transpose(Y) 
 
        # FUNCION DE DESLIZAMIENTO 
        self.s = self.dqtilde + self.Lambda*self.qtilde

        if self.s[0,0]<self.s_lim and self.s[0,0]>-self.s_lim:
            self.ae = self.ae - self.T*self.Gamma*Y_trans@self.s


        # LEY DE CONTROL 
        self.u=Y@self.ae-self.Kd*self.s

        # Limitador de corriente:
        if self.u[0,0] >= self.u_lim:
            self.u[0,0] = self.u_lim
        elif self.u[0,0] <= -self.u_lim:
            self.u[0,0] = -self.u_lim

        if self.u[1,0] >= self.u_lim:
            self.u[1,0] = self.u_lim
        elif self.u[1,0] <= -self.u_lim:
            self.u[1,0] = -self.u_lim
        


def main():
    rospy.init_node('ac_control')
    freq = 100
    rate = rospy.Rate(freq) #10 Hz
    control=controler(freq)

    while not rospy.is_shutdown():
        control.cb_control()
        control.publish_present_u()
        U=np.round(control.u,4)
        #print("U1:",control.u[0],"\n")
        #print("\n ------- \n")
        #print("U2:",control.u[1],"\n")
        rospy.loginfo(str(U))
        rate.sleep()

if __name__ == "__main__":
    main()