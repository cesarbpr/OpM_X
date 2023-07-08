#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray


class controler:
    # CONTROL ADAPTATIVO PARA ROBOT DE 2GDL
    # FUNCION DE CONTROL ADAPTATIVO QUE DEVUELVE LAS SENALES DE LAS 2
    # ARTICULACIONES, SENALES DE CONTROL Y POSICION

    # LAS ENTRADAS SON LOS PARAMETROS DE SINTONIZACION
    k1=1
    ld=20
    kd=0.5
    kk=3
    # PARAMETROS DE CONTROL
    I = np.eye(2,2)
    K  = kk*I
    Kd = kd*I
    K1 = k1*I
    Ld = ld*I
    u_lim=2
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
    #   self.Jeq=(n**2)*Jm+Jg;self.Beq=(n**2)*Bm+Bg;
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
    # Keq=self.Beq+(n**2)*Km*Kb/Ra;

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
    u =  np.array([[0], [0]], dtype=np.float64)
    e = np.array([[0], [0]], dtype=np.float64)
    dqtilde =  np.array([[0], [0]], dtype=np.float64)
    u_pas=np.array([[0], [0]], dtype=np.float64)


    def __init__(self,sample_time):
        pub_topic_u="u"
        sub_topic_q_des="q_des"
        sub_topic_q_real="pos_present_value"        
        #pub_topic_u="u"
        #sub_topic_q_des="q_des"
        #sub_topic_q_real="q_real"
        #Timepo de muestreo
        self.T=1/sample_time

        self.pub_u=rospy.Publisher(pub_topic_u,Float64MultiArray,queue_size=10)
        self.qd_suscriber=rospy.Subscriber(sub_topic_q_des,Float64MultiArray,self.cb_qdes_in)
        self.qreal_suscriber=rospy.Subscriber(sub_topic_q_real,Float64MultiArray,self.cb_qreal_in)
    
    """ # Con el robot real
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
        q1=q_real.data[0]
        q2=q_real.data[1]
        self.q = np.array([[q1], [q2]])

    def publish_present_u(self):
        pub_array = Float64MultiArray()
        pub_array.data = [0,0]
        pub_array.data[0] = self.u[0,0]
        pub_array.data[1] = self.u[1,0]
        self.pub_u.publish(pub_array)

    def cb_qdes_in(self,q_des):
        q_des1=q_des.data[0]
        q_des2=q_des.data[1]
        self.qd = np.array([[q_des1], [q_des2]])



    def en_control(self):
        
        # ERROR TRACKING
        e = self.q - self.qd
        z1 = e

        dqr = self.dqd - self.K@z1

        # OBSERVADOR DE VELOCIDAD
        self.dqe = self.dqd + self.Ld@(self.q - self.qe)
        self.qe = self.qe + self.T*self.dqe
        
        # DINAMICA DEL MANIPULADOR
        # Matriz Ge inercias:
        M11e=(self.U_lin)*(self.Iyy3 + self.Iyy4 + self.Jeq + self.m4*(self.L3_y**2 + 2*np.cos(self.qe[1,0])*self.L3_y*self.l4_y + self.l4_y**2) + self.l3_y**2*self.m3)
        M12e=(self.U_lin)*(self.Iyy4 + self.m4*(self.l4_y**2 + self.L3_y*np.cos(self.qe[1,0])*self.l4_y))
        M21e=(self.U_lin)*(self.Iyy4 + self.m4*(self.l4_y**2 + self.L3_y*np.cos(self.qe[1,0])*self.l4_y))
        M22e=(self.U_lin)*(self.m4*self.l4_y**2 + self.Iyy4 + self.Jeq)
        Me = [[M11e  ,  M12e] , [M21e ,   M22e] ]
        ##########
        # Matriz Ge coriolis y fuerzas centrífugas:
        h=-self.L3_y*self.l4_y*self.m4*np.sin(self.qe[1,0])
        P11e=(self.U_lin)*(self.Beq + h*self.dqe[1,0])
        P12e=(self.U_lin)*(h*(self.dqe[1,0] + self.dqe[1,0]))
        P21e=(self.U_lin)*(-h*self.dqe[1,0])
        P22e=(self.U_lin)*(self.Beq)
        Pe  = [[P11e,P12e],[P21e,P22e]]
    ##############
        # Vector de gravedad
        G1e = (self.U_lin)*(- self.g*self.m4*(self.l4_y*np.cos(self.qe[0,0] + self.qe[1,0]) + self.L3_y*np.cos(self.qe[0,0])) - self.g*self.l3_y*self.m3*np.cos(self.qe[0,0]))
        G2e = (self.U_lin)*(-self.g*self.l4_y*self.m4*np.cos(self.qe[0,0] + self.qe[1,0]))

        Ge = [[G1e], [G2e]];
        
        # LEY DE CONTROL 
        self.u =(Me@self.ddqd + Pe@dqr + Ge - self.Kd@(self.dqe - dqr) - self.K1@z1)

        # Limitador de corriente:
        if self.u[0,0] >= self.u_lim or self.u[0,0] <= -self.u_lim:
            self.u[0,0] = self.u_pas[0,0]
        else:
            self.u_pas[0,0] = self.u[0,0]

        if self.u[1,0] >= self.u_lim or self.u[1,0] <= -self.u_lim:
            self.u[1,0] = self.u_pas[1,0]
        else:
            self.u_pas[1,0] = self.u[1,0]
        


def main():
    rospy.init_node('bs_control')
    freq = 100
    rate = rospy.Rate(freq) #10 Hz
    control=controler(freq)

    while not rospy.is_shutdown():
        control.en_control()
        control.publish_present_u()
        U=np.round(control.u,4)
        #print("U1:",control.u[0],"\n")
        #print("\n ------- \n")
        #print("U2:",control.u[1],"\n")
        rospy.loginfo(str(U))
        rate.sleep()

if __name__ == "__main__":
    main()