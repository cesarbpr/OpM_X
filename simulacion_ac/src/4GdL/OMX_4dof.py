#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

class robot:

    # DATOS DEL SUB-SISTEMA MECANICO
    # Masas
    m1 = 0.105
    m2 = 0.142
    m3=0.135
    m4=0.236
    #Longitudes de los eslabones
    l1_z = 0.0266
    L1_z=0.043
    L0_z=0.034
    l2_y=0.106
    L2_x=-0.024
    L2_y=0.128
    l3_y=0.0933
    L3_y=0.124
    l4_y=0.060
    L4_y=0.126
    # Inercias de eslabones y motores
    Ixx1=107819*1e-09
    Iyy1=103553*1e-09
    Izz1=17644*1e-09
    Ixx2=1836523*1e-09
    Iyy2=1856215*1e-09
    Izz2=54940*1e-09
    Ixx3=1358961*1e-09
    Iyy3=1350228*1e-09
    Izz3=24835*1e-09
    Ixx4=1128335*1e-09
    Iyy4=1077791*1e-09
    Izz4=182774*1e-09
    Jm=0.0000071
    Jg=0.0000053
    # Jeq=(n**2)*Jm+Jg

    #Gravedad y relacion de trnasmision
    #   n=353.5
    n=353.5
    g=-9.81
    #Constantes de friccion motores:
    # Bm=0.0001 Bg=0.01 
    # Beq=(n**2)*Bm+Bg
    Jeq=0.0037
    Beq=0.012
    # DATOS DEL SUB-SISTEMA ELECTRICO
    #   Km=0.0458 Kb=0.0458 KA=8.5 Ra=2.49Ka=8.5
    Km=0.005
    Kb=1
    KA=1
    Ra=1
    Ka=1  
    # Keq=Beq+(n**2)*Km*Kb/Ra
    Keq=Beq
    U_lin=1/(Km*n)

    # Posción y velocidad inicial OMX
    q = np.array([[np.deg2rad(0)], [np.deg2rad(0)], [np.deg2rad(0)], [np.deg2rad(0)]], dtype=np.float64)
    dq = np.array([[0], [0], [0], [0]], dtype=np.float64)
    q_ac=np.array([0,0,0,0], dtype=np.float64)
    u = np.array([[0], [0], [0], [0]], dtype=np.float64)

    def __init__(self,sample_time):
        pub_topic_q_real="pos_present_value"
        sub_topic_u="u"

        #Timepo de muestreo
        self.T=1/sample_time

        self.pub_qreal=rospy.Publisher(pub_topic_q_real,Float64MultiArray,queue_size=10)
        self.u_suscriber=rospy.Subscriber(sub_topic_u,Float64MultiArray,self.cb_u)

    def publish_present_q(self):
        pub_array = Float64MultiArray()
        pub_array.data  = [0,0,0,0]
        pub_array.data[0] = self.q[0,0]
        pub_array.data[1] = self.q[1,0]
        pub_array.data[2] = self.q[2,0]
        pub_array.data[3] = self.q[3,0]
        self.pub_qreal.publish(pub_array)
    
    def cb_u(self, u_data):
        u1=u_data.data[0]
        u2=u_data.data[1]
        u3=u_data.data[2]
        u4=u_data.data[3]
        self.u = np.array([[u1], [u2], [u3], [u4]])

    def OMX_modelo(self):

        # MODELO DINAMICO DEL SISTEMA 
        M11 = (self.Jeq+self.Ixx3 + self.Ixx4 + self.Izz1 + self.Izz2 + self.m3*(self.l3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]))**2 - self.Ixx4*np.sin(self.q[1,0] + self.q[2,0] + self.q[3,0])**2 + self.Izz4*np.sin(self.q[1,0] + self.q[2,0] + self.q[3,0])**2 - self.Ixx3*np.sin(self.q[1,0] + self.q[2,0])**2 + self.Izz3*np.sin(self.q[1,0] + self.q[2,0])**2 + self.m4*(self.L3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]) + self.l4_y*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0]))**2 + self.Ixx2*np.sin(self.q[1,0])**2 - self.Izz2*np.sin(self.q[1,0])**2 + self.l2_y**2*self.m2*np.sin(self.q[1,0])**2)

        M22 = (self.Jeq+self.Iyy2 + self.Iyy3 + self.Iyy4 + self.L2_x**2*self.m3 + self.L2_x**2*self.m4 + self.L2_y**2*self.m3 + self.L2_y**2*self.m4 + self.L3_y**2*self.m4 + self.l2_y**2*self.m2 + self.l3_y**2*self.m3 + self.l4_y**2*self.m4 + 2*self.L2_x*self.l4_y*self.m4*np.cos(self.q[2,0] + self.q[3,0]) + 2*self.L2_x*self.L3_y*self.m4*np.cos(self.q[2,0]) + 2*self.L2_y*self.l4_y*self.m4*np.sin(self.q[2,0] + self.q[3,0]) + 2*self.L2_y*self.L3_y*self.m4*np.sin(self.q[2,0]) + 2*self.L2_x*self.l3_y*self.m3*np.cos(self.q[2,0]) + 2*self.L3_y*self.l4_y*self.m4*np.cos(self.q[3,0]) + 2*self.L2_y*self.l3_y*self.m3*np.sin(self.q[2,0]))

        M23= (self.Iyy3 + self.Iyy4 + self.L3_y**2*self.m4 + self.l3_y**2*self.m3 + self.l4_y**2*self.m4 + self.L2_x*self.l4_y*self.m4*np.cos(self.q[2,0] + self.q[3,0]) + self.L2_x*self.L3_y*self.m4*np.cos(self.q[2,0]) + self.L2_y*self.l4_y*self.m4*np.sin(self.q[2,0] + self.q[3,0]) + self.L2_y*self.L3_y*self.m4*np.sin(self.q[2,0]) + self.L2_x*self.l3_y*self.m3*np.cos(self.q[2,0]) + 2*self.L3_y*self.l4_y*self.m4*np.cos(self.q[3,0]) + self.L2_y*self.l3_y*self.m3*np.sin(self.q[2,0]))

        M24 = (self.Iyy4 + self.l4_y*self.m4*(self.l4_y + self.L2_x*np.cos(self.q[2,0] + self.q[3,0]) + self.L2_y*np.sin(self.q[2,0] + self.q[3,0]) + self.L3_y*np.cos(self.q[3,0])))

        M32 = (self.Iyy3 + self.Iyy4 + self.L3_y**2*self.m4 + self.l3_y**2*self.m3 + self.l4_y**2*self.m4 + self.L2_x*self.l4_y*self.m4*np.cos(self.q[2,0] + self.q[3,0]) + self.L2_x*self.L3_y*self.m4*np.cos(self.q[2,0]) + self.L2_y*self.l4_y*self.m4*np.sin(self.q[2,0] + self.q[3,0]) + self.L2_y*self.L3_y*self.m4*np.sin(self.q[2,0]) + self.L2_x*self.l3_y*self.m3*np.cos(self.q[2,0]) + 2*self.L3_y*self.l4_y*self.m4*np.cos(self.q[3,0]) + self.L2_y*self.l3_y*self.m3*np.sin(self.q[2,0]))

        M33 =  (self.Jeq+self.Iyy3 + self.Iyy4 + self.m4*(self.L3_y**2 + 2*np.cos(self.q[3,0])*self.L3_y*self.l4_y + self.l4_y**2) + self.l3_y**2*self.m3)

        M34 = (self.Iyy4 + self.m4*(self.l4_y**2 + self.L3_y*np.cos(self.q[3,0])*self.l4_y))

        M42= (self.Iyy4 + self.l4_y*self.m4*(self.l4_y + self.L2_x*np.cos(self.q[2,0] + self.q[3,0]) + self.L2_y*np.sin(self.q[2,0] + self.q[3,0]) + self.L3_y*np.cos(self.q[3,0])))

        M43=  (self.Iyy4 + self.m4*(self.l4_y**2 + self.L3_y*np.cos(self.q[3,0])*self.l4_y))

        M44=  (self.Jeq+ self.m4*self.l4_y**2 + self.Iyy4)

        M = self.U_lin*np.array([[M11  ,  0   ,     0   ,    0] ,
                                [0 ,   M22  ,   M23   ,M24],
                                [0   , M32  ,   M33 ,  M34],
                                [0  ,  M42   ,  M43 ,  M44]])
        ############
        P11 = (self.Keq - self.dq[2,0]*((self.Ixx3*np.sin(2*self.q[1,0] + 2*self.q[2,0]))/2 - (self.Izz3*np.sin(2*self.q[1,0] + 2*self.q[2,0]))/2 + (self.Ixx4*np.sin(2*self.q[1,0] + 2*self.q[2,0] + 2*self.q[3,0]))/2 - (self.Izz4*np.sin(2*self.q[1,0] + 2*self.q[2,0] + 2*self.q[3,0]))/2 + self.m4*(self.L3_y*np.sin(self.q[1,0] + self.q[2,0]) + self.l4_y*np.sin(self.q[1,0] + self.q[2,0] + self.q[3,0]))*(self.L3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]) + self.l4_y*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0])) + self.l3_y*self.m3*np.sin(self.q[1,0] + self.q[2,0])*(self.l3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]))) - self.dq[1,0]*((self.Ixx3*np.sin(2*self.q[1,0] + 2*self.q[2,0]))/2 - (self.Izz3*np.sin(2*self.q[1,0] + 2*self.q[2,0]))/2 - (self.Ixx2*np.sin(2*self.q[1,0]))/2 + (self.Izz2*np.sin(2*self.q[1,0]))/2 + (self.Ixx4*np.sin(2*self.q[1,0] + 2*self.q[2,0] + 2*self.q[3,0]))/2 - (self.Izz4*np.sin(2*self.q[1,0] + 2*self.q[2,0] + 2*self.q[3,0]))/2 + self.m4*(self.L3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]) + self.l4_y*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0]))*(self.L3_y*np.sin(self.q[1,0] + self.q[2,0]) + self.L2_y*np.cos(self.q[1,0]) + self.L2_x*np.sin(self.q[1,0]) + self.l4_y*np.sin(self.q[1,0] + self.q[2,0] + self.q[3,0])) + self.m3*(self.l3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]))*(self.L2_y*np.cos(self.q[1,0]) + self.l3_y*np.sin(self.q[1,0] + self.q[2,0]) + self.L2_x*np.sin(self.q[1,0])) - (self.l2_y**2*self.m2*np.sin(2*self.q[1,0]))/2) - self.dq[3,0]*np.sin(self.q[1,0] + self.q[2,0] + self.q[3,0])*(self.Ixx4*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0]) - self.Izz4*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0]) + self.l4_y**2*self.m4*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0]) + self.L3_y*self.l4_y*self.m4*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*self.l4_y*self.m4*np.cos(self.q[1,0]) - self.L2_y*self.l4_y*self.m4*np.sin(self.q[1,0])))

        P12 = (-self.dq[0,0]*((self.Ixx3*np.sin(2*self.q[1,0] + 2*self.q[2,0]))/2 - (self.Izz3*np.sin(2*self.q[1,0] + 2*self.q[2,0]))/2 - (self.Ixx2*np.sin(2*self.q[1,0]))/2 + (self.Izz2*np.sin(2*self.q[1,0]))/2 + (self.Ixx4*np.sin(2*self.q[1,0] + 2*self.q[2,0] + 2*self.q[3,0]))/2 - (self.Izz4*np.sin(2*self.q[1,0] + 2*self.q[2,0] + 2*self.q[3,0]))/2 + self.m4*(self.L3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]) + self.l4_y*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0]))*(self.L3_y*np.sin(self.q[1,0] + self.q[2,0]) + self.L2_y*np.cos(self.q[1,0]) + self.L2_x*np.sin(self.q[1,0]) + self.l4_y*np.sin(self.q[1,0] + self.q[2,0] + self.q[3,0])) + self.m3*(self.l3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]))*(self.L2_y*np.cos(self.q[1,0]) + self.l3_y*np.sin(self.q[1,0] + self.q[2,0]) + self.L2_x*np.sin(self.q[1,0])) - (self.l2_y**2*self.m2*np.sin(2*self.q[1,0]))/2))

        P13 = (-self.dq[0,0]*((self.Ixx3*np.sin(2*self.q[1,0] + 2*self.q[2,0]))/2 - (self.Izz3*np.sin(2*self.q[1,0] + 2*self.q[2,0]))/2 + (self.Ixx4*np.sin(2*self.q[1,0] + 2*self.q[2,0] + 2*self.q[3,0]))/2 - (self.Izz4*np.sin(2*self.q[1,0] + 2*self.q[2,0] + 2*self.q[3,0]))/2 + self.m4*(self.L3_y*np.sin(self.q[1,0] + self.q[2,0]) + self.l4_y*np.sin(self.q[1,0] + self.q[2,0] + self.q[3,0]))*(self.L3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]) + self.l4_y*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0])) + self.l3_y*self.m3*np.sin(self.q[1,0] + self.q[2,0])*(self.l3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]))))

        P14 = (-self.dq[0,0]*np.sin(self.q[1,0] + self.q[2,0] + self.q[3,0])*(self.Ixx4*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0]) - self.Izz4*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0]) + self.l4_y**2*self.m4*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0]) + self.L3_y*self.l4_y*self.m4*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*self.l4_y*self.m4*np.cos(self.q[1,0]) - self.L2_y*self.l4_y*self.m4*np.sin(self.q[1,0])))

        P21 = -P12

        P22 = (self.Keq + self.dq[2,0]*(self.L2_y*self.l4_y*self.m4*np.cos(self.q[2,0] + self.q[3,0]) + self.L2_y*self.L3_y*self.m4*np.cos(self.q[2,0]) - self.L2_x*self.l4_y*self.m4*np.sin(self.q[2,0] + self.q[3,0]) - self.L2_x*self.L3_y*self.m4*np.sin(self.q[2,0]) + self.L2_y*self.l3_y*self.m3*np.cos(self.q[2,0]) - self.L2_x*self.l3_y*self.m3*np.sin(self.q[2,0])) - self.l4_y*self.m4*self.dq[3,0]*(self.L2_x*np.sin(self.q[2,0] + self.q[3,0]) - self.L2_y*np.cos(self.q[2,0] + self.q[3,0]) + self.L3_y*np.sin(self.q[3,0])))

        P23 = (self.L2_y*self.l4_y*self.m4*self.dq[1,0]*np.cos(self.q[2,0] + self.q[3,0]) + self.L2_y*self.l4_y*self.m4*self.dq[2,0]*np.cos(self.q[2,0] + self.q[3,0]) + self.L2_y*self.l4_y*self.m4*self.dq[3,0]*np.cos(self.q[2,0] + self.q[3,0]) + self.L2_y*self.L3_y*self.m4*self.dq[1,0]*np.cos(self.q[2,0]) + self.L2_y*self.L3_y*self.m4*self.dq[2,0]*np.cos(self.q[2,0]) - self.L2_x*self.l4_y*self.m4*self.dq[1,0]*np.sin(self.q[2,0] + self.q[3,0]) - self.L2_x*self.l4_y*self.m4*self.dq[2,0]*np.sin(self.q[2,0] + self.q[3,0]) - self.L2_x*self.l4_y*self.m4*self.dq[3,0]*np.sin(self.q[2,0] + self.q[3,0]) - self.L2_x*self.L3_y*self.m4*self.dq[1,0]*np.sin(self.q[2,0]) - self.L2_x*self.L3_y*self.m4*self.dq[2,0]*np.sin(self.q[2,0]) + self.L2_y*self.l3_y*self.m3*self.dq[1,0]*np.cos(self.q[2,0]) + self.L2_y*self.l3_y*self.m3*self.dq[2,0]*np.cos(self.q[2,0]) - self.L2_x*self.l3_y*self.m3*self.dq[1,0]*np.sin(self.q[2,0]) - self.L2_x*self.l3_y*self.m3*self.dq[2,0]*np.sin(self.q[2,0]) - self.L3_y*self.l4_y*self.m4*self.dq[3,0]*np.sin(self.q[3,0]))

        P24 = (-self.l4_y*self.m4*(self.L2_x*np.sin(self.q[2,0] + self.q[3,0]) - self.L2_y*np.cos(self.q[2,0] + self.q[3,0]) + self.L3_y*np.sin(self.q[3,0]))*(self.dq[1,0] + self.dq[2,0] + self.dq[3,0]))

        P31 = -P13

        P32 = (self.L2_x*self.l4_y*self.m4*self.dq[1,0]*np.sin(self.q[2,0] + self.q[3,0]) - self.L2_y*self.L3_y*self.m4*self.dq[1,0]*np.cos(self.q[2,0]) - self.L2_y*self.l4_y*self.m4*self.dq[1,0]*np.cos(self.q[2,0] + self.q[3,0]) + self.L2_x*self.L3_y*self.m4*self.dq[1,0]*np.sin(self.q[2,0]) - self.L2_y*self.l3_y*self.m3*self.dq[1,0]*np.cos(self.q[2,0]) + self.L2_x*self.l3_y*self.m3*self.dq[1,0]*np.sin(self.q[2,0]) - self.L3_y*self.l4_y*self.m4*self.dq[3,0]*np.sin(self.q[3,0]))

        P33 = (self.Keq -self.L3_y*self.l4_y*self.m4*self.dq[3,0]*np.sin(self.q[3,0]))

        P34 = (-self.L3_y*self.l4_y*self.m4*np.sin(self.q[3,0])*(self.dq[1,0] + self.dq[2,0] + self.dq[3,0]))

        P41 = -P14

        P42 = (self.l4_y*self.m4*(self.L3_y*self.dq[1,0]*np.sin(self.q[3,0]) + self.L3_y*self.dq[2,0]*np.sin(self.q[3,0]) - self.L2_y*self.dq[1,0]*np.cos(self.q[2,0] + self.q[3,0]) + self.L2_x*self.dq[1,0]*np.sin(self.q[2,0] + self.q[3,0])))

        P43 = (self.L3_y*self.l4_y*self.m4*np.sin(self.q[3,0])*(self.dq[1,0] + self.dq[2,0]))

        P44 = (self.Keq)

        P  = self.U_lin*np.array([[P11, P12, P13, P14], 
                    [P21, P22, P23, P24], 
                    [P31, P32, P33, P34],
                    [P41, P42, P43, P44]])

        ##########

        D2 = (self.g*self.l2_y*self.m2*np.sin(self.q[1,0]) - self.g*self.m4*(self.L3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0]) + self.l4_y*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0])) - self.g*self.m3*(self.l3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.L2_x*np.cos(self.q[1,0]) - self.L2_y*np.sin(self.q[1,0])))
        D3 = (- self.g*self.m4*(self.L3_y*np.cos(self.q[1,0] + self.q[2,0]) + self.l4_y*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0])) - self.g*self.l3_y*self.m3*np.cos(self.q[1,0] + self.q[2,0]))
        D4 = (-self.g*self.l4_y*self.m4*np.cos(self.q[1,0] + self.q[2,0] + self.q[3,0]))

        d = self.U_lin*np.array([[0], [D2], [D3], [D4]])

        self.dq = self.dq + self.T * np.linalg.solve(M, self.u - np.dot(P, self.dq) - d)
        self.q = self.q + self.T * self.dq
        self.q_ac=([self.q[0,0],self.q[1,0],self.q[2,0],self.q[3,0]])


def main():
    rospy.init_node('OMX_plant')
    freq = 100 
    rate = rospy.Rate(freq) #10 Hz
    OMX=robot(freq)

    while not rospy.is_shutdown():
        OMX.OMX_modelo()
        OMX.publish_present_q()
        q_re=np.round(np.rad2deg(OMX.q_ac),2)
        #print("q1:",OMX.self.q[0],"\n")
        #print("\n ------- \n")
        #print("q2:",OMX.self.q[1],"\n")
        rospy.loginfo(str(q_re))
        rate.sleep()

if __name__ == "__main__":
    main()