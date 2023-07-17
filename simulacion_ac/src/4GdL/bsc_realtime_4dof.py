#!/usr/bin/env python3
import rospy
import numpy as np
import csv
import os
from std_msgs.msg import Float64MultiArray


class controler:
    # CONTROL ADAPTATIVO PARA ROBOT DE 2GDL
    # FUNCION DE CONTROL ADAPTATIVO QUE DEVUELVE LAS SENALES DE LAS 2
    # ARTICULACIONES, SENALES DE CONTROL Y POSICION

    # LAS ENTRADAS SON LOS PARAMETROS DE SINTONIZACION
    # Funcionan
    ''' # para referencia
    k1=1.2
    ld=40
    kd=0.08
    kk=1
    ''' 
    '''
    # Funcionan para seguimiento
    k1=1.2
    ld=90 #Afecta a la respuesta del sistema
    kd=0.07 #afecta a la estabilidad del sistema. !! Si es muy alto muere
    kk=1.5
    '''
    k1=1
    ld=30 #Afecta a la respuesta del sistema
    kd=0.07 #afecta a la estabilidad del sistema. !! Si es muy alto muere
    kk=1.5
    k1_u2 = 3
    kd_u2 =0.01
    # PARAMETROS DE CONTROL
    I = np.eye(4,4)
    K  = kk*I
    Kd = kd*I
    K1 = k1*I
    K1_u2 = k1_u2*I
    Kd_u2 = kd_u2*I
    Ld = ld*I
    #tap=1
    u_lim=0.672

    # DATOS DEL SUB-SISTEMA MECANICO
    # Masas
    m2 = 0.142
    m3 = 0.135
    m4 = 0.236
    #m4 = 0.1773
    #Longitudes de los eslabones

    l2_y=0.106
    L2_x=-0.024
    L2_y=0.128
    l3_y=0.0933
    L3_y=0.124
    l4_y=0.060

    # Inercias de eslabones y motores
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
    g=9.81
    #Constantes de friccion motores:
    # Bm=0.0001 Bg=0.01 
    # Beq=(n**2)*Bm+Bg
    #Jeq=0.0037
    #Beq=0.0012
    Jeq=0.0037
    Beq=0.12
    # DATOS DEL SUB-SISTEMA ELECTRICO
    #   Km=0.0458 Kb=0.0458 KA=8.5 Ra=2.49Ka=8.5
    #Km=0.005
    Km=0.0055
    Kb=1
    KA=1
    Ra=1
    Ka=1  
    # Keq=Beq+(n**2)*Km*Kb/Ra
    Keq=Beq
    U_lin=1/(Km*n)

    # CONDICIONES INICIALES
    # VALORES INICIALES
    q = np.array([[np.deg2rad(0)], [np.deg2rad(-20)],[np.deg2rad(0)],[np.deg2rad(-70)]], dtype=np.float64)
    dq = np.array([[0], [0],[0], [0]], dtype=np.float64)
    qe= np.array([[np.deg2rad(0)], [np.deg2rad(-20)],[np.deg2rad(0)],[np.deg2rad(-70)]], dtype=np.float64) 
    e=np.array([[0], [0], [0], [0]], dtype=np.float64)
    dqe=np.array([[0], [0],[0], [0]], dtype=np.float64)
    dqd = np.array([[0], [0],[0], [0]], dtype=np.float64)
    ddqd = np.array([[0], [0],[0], [0]], dtype=np.float64)
    qd=np.array([[0], [0],[0], [0]], dtype=np.float64)
    u =  np.array([[0], [0], [0], [0]], dtype=np.float64)
    u_sig = np.array([0,0,0,0], dtype=np.float64)


    def __init__(self,sample_time):
        pub_topic_u="u"
        pub_topic_dqe="dqe"
        sub_topic_q_des="q_des"
        sub_topic_q_real="pos_present_value"  
        sub_topic_q_vel_real = "vel_present_value "     
        self.T=1/sample_time

        self.pub_u=rospy.Publisher(pub_topic_u,Float64MultiArray,queue_size=10)
        self.pub_dqe=rospy.Publisher(pub_topic_dqe,Float64MultiArray,queue_size=10)
        self.qd_suscriber=rospy.Subscriber(sub_topic_q_des,Float64MultiArray,self.cb_qdes_in)
        self.qreal_suscriber=rospy.Subscriber(sub_topic_q_real,Float64MultiArray,self.cb_qreal_in)
        self.qreal_suscriber=rospy.Subscriber(sub_topic_q_vel_real,Float64MultiArray,self.cb_q_vel_in)

    # Obtención de la posición real
    def cb_qreal_in(self,q_real):
        q1=q_real.data[0]
        q2=q_real.data[1]
        q3=q_real.data[2]
        q4=q_real.data[3]
        self.q = np.array([[q1], [q2], [q3], [q4]])

    # Obtención de la velocidad real
    def cb_q_vel_in(self,q_vel_real):
        qv1=q_vel_real.data[0]
        qv2=q_vel_real.data[1]
        qv3=q_vel_real.data[2]
        qv4=q_vel_real.data[3]
        self.dq = np.array([[qv1], [qv2], [qv3], [qv4]])

    # Publicando la señal de control
    def publish_present_u(self):
        pub_array = Float64MultiArray()
        pub_array.data = [0,0,0,0,0]
        pub_array.data[0] = self.u[0,0]
        pub_array.data[1] = self.u[1,0]
        pub_array.data[2] = self.u[2,0]
        pub_array.data[3] = self.u[3,0]
        self.pub_u.publish(pub_array)

    # Publicando la velocidad estimada
    def publish_present_dqe(self):
        pub_array = Float64MultiArray()
        pub_array.data = [0,0,0,0,0]
        pub_array.data[0] = self.dqe[0,0]
        pub_array.data[1] = self.dqe[1,0]
        pub_array.data[2] = self.dqe[2,0]
        pub_array.data[3] = self.dqe[3,0]
        self.pub_dqe.publish(pub_array)

    # Suscribiendo a la posición deseada
    def cb_qdes_in(self,q_des):
        q_des1=q_des.data[0]
        q_des2=q_des.data[1]
        q_des3=q_des.data[2]
        q_des4=q_des.data[3]
        self.qd = np.array([[q_des1], [q_des2], [q_des3], [q_des4]])

    def csv_getData(self):
        current_dir = "/home/cesar/OpM_X/catkin_ws/src/simulacion_ac/src/4GdL/DataLog"
        array_q = self.q.tolist()
        current_time = rospy.Time.now()
        file_path = os.path.join(current_dir, 'Dato_simu_4GdL.csv')
        with open(file_path,'a') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([current_time] + array_q)
    def en_control(self):
        
        # ERROR TRACKING
        e = self.q - self.qd
        z1 = e

        dqr = self.dqd - self.K@z1

        # OBSERVADOR DE VELOCIDAD
        
        self.dqe = self.dqd + self.Ld@(self.q - self.qe)
        self.qe = self.qe + self.T*self.dqe
        '''
        self.dqe = self.dq
        self.qe = self.q
        '''
        # DINAMICA DEL MANIPULADOR
        # atriz de Inercias
        M11e = (self.Jeq+self.Ixx3 + self.Ixx4 + self.Izz1 + self.Izz2 + self.m3*(self.l3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]))**2 - self.Ixx4*np.sin(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])**2 + self.Izz4*np.sin(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])**2 - self.Ixx3*np.sin(self.qe[1,0] + self.qe[2,0])**2 + self.Izz3*np.sin(self.qe[1,0] + self.qe[2,0])**2 + self.m4*(self.L3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]) + self.l4_y*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]))**2 + self.Ixx2*np.sin(self.qe[1,0])**2 - self.Izz2*np.sin(self.qe[1,0])**2 + self.l2_y**2*self.m2*np.sin(self.qe[1,0])**2)

        M22e = (self.Jeq+self.Iyy2 + self.Iyy3 + self.Iyy4 + self.L2_x**2*self.m3 + self.L2_x**2*self.m4 + self.L2_y**2*self.m3 + self.L2_y**2*self.m4 + self.L3_y**2*self.m4 + self.l2_y**2*self.m2 + self.l3_y**2*self.m3 + self.l4_y**2*self.m4 + 2*self.L2_x*self.l4_y*self.m4*np.cos(self.qe[2,0] + self.qe[3,0]) + 2*self.L2_x*self.L3_y*self.m4*np.cos(self.qe[2,0]) + 2*self.L2_y*self.l4_y*self.m4*np.sin(self.qe[2,0] + self.qe[3,0]) + 2*self.L2_y*self.L3_y*self.m4*np.sin(self.qe[2,0]) + 2*self.L2_x*self.l3_y*self.m3*np.cos(self.qe[2,0]) + 2*self.L3_y*self.l4_y*self.m4*np.cos(self.qe[3,0]) + 2*self.L2_y*self.l3_y*self.m3*np.sin(self.qe[2,0]))

        M23e = (self.Iyy3 + self.Iyy4 + self.L3_y**2*self.m4 + self.l3_y**2*self.m3 + self.l4_y**2*self.m4 + self.L2_x*self.l4_y*self.m4*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_x*self.L3_y*self.m4*np.cos(self.qe[2,0]) + self.L2_y*self.l4_y*self.m4*np.sin(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.L3_y*self.m4*np.sin(self.qe[2,0]) + self.L2_x*self.l3_y*self.m3*np.cos(self.qe[2,0]) + 2*self.L3_y*self.l4_y*self.m4*np.cos(self.qe[3,0]) + self.L2_y*self.l3_y*self.m3*np.sin(self.qe[2,0]))

        M24e = (self.Iyy4 + self.l4_y*self.m4*(self.l4_y + self.L2_x*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*np.sin(self.qe[2,0] + self.qe[3,0]) + self.L3_y*np.cos(self.qe[3,0])))

        M32e = (self.Iyy3 + self.Iyy4 + self.L3_y**2*self.m4 + self.l3_y**2*self.m3 + self.l4_y**2*self.m4 + self.L2_x*self.l4_y*self.m4*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_x*self.L3_y*self.m4*np.cos(self.qe[2,0]) + self.L2_y*self.l4_y*self.m4*np.sin(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.L3_y*self.m4*np.sin(self.qe[2,0]) + self.L2_x*self.l3_y*self.m3*np.cos(self.qe[2,0]) + 2*self.L3_y*self.l4_y*self.m4*np.cos(self.qe[3,0]) + self.L2_y*self.l3_y*self.m3*np.sin(self.qe[2,0]))

        M33e =  (self.Jeq+self.Iyy3 + self.Iyy4 + self.m4*(self.L3_y**2 + 2*np.cos(self.qe[3,0])*self.L3_y*self.l4_y + self.l4_y**2) + self.l3_y**2*self.m3)

        M34e = (self.Iyy4 + self.m4*(self.l4_y**2 + self.L3_y*np.cos(self.qe[3,0])*self.l4_y))

        M42e= (self.Iyy4 + self.l4_y*self.m4*(self.l4_y + self.L2_x*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*np.sin(self.qe[2,0] + self.qe[3,0]) + self.L3_y*np.cos(self.qe[3,0])))

        M43e=  (self.Iyy4 + self.m4*(self.l4_y**2 + self.L3_y*np.cos(self.qe[3,0])*self.l4_y))

        M44e=  (self.Jeq+ self.m4*self.l4_y**2 + self.Iyy4)

        Me = self.U_lin*np.array([[M11e  ,  0   ,     0   ,    0] ,
                            [0 ,   M22e  ,   M23e   ,M24e],
                            [0   , M32e  ,   M33e ,  M34e],
                            [0  ,  M42e   ,  M43e ,  M44e]])
        ############
        P11e = (self.Keq - self.dqe[2,0]*((self.Ixx3*np.sin(2*self.qe[1,0] + 2*self.qe[2,0]))/2 - (self.Izz3*np.sin(2*self.qe[1,0] + 2*self.qe[2,0]))/2 + (self.Ixx4*np.sin(2*self.qe[1,0] + 2*self.qe[2,0] + 2*self.qe[3,0]))/2 - (self.Izz4*np.sin(2*self.qe[1,0] + 2*self.qe[2,0] + 2*self.qe[3,0]))/2 + self.m4*(self.L3_y*np.sin(self.qe[1,0] + self.qe[2,0]) + self.l4_y*np.sin(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]))*(self.L3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]) + self.l4_y*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])) + self.l3_y*self.m3*np.sin(self.qe[1,0] + self.qe[2,0])*(self.l3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]))) - self.dqe[1,0]*((self.Ixx3*np.sin(2*self.qe[1,0] + 2*self.qe[2,0]))/2 - (self.Izz3*np.sin(2*self.qe[1,0] + 2*self.qe[2,0]))/2 - (self.Ixx2*np.sin(2*self.qe[1,0]))/2 + (self.Izz2*np.sin(2*self.qe[1,0]))/2 + (self.Ixx4*np.sin(2*self.qe[1,0] + 2*self.qe[2,0] + 2*self.qe[3,0]))/2 - (self.Izz4*np.sin(2*self.qe[1,0] + 2*self.qe[2,0] + 2*self.qe[3,0]))/2 + self.m4*(self.L3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]) + self.l4_y*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]))*(self.L3_y*np.sin(self.qe[1,0] + self.qe[2,0]) + self.L2_y*np.cos(self.qe[1,0]) + self.L2_x*np.sin(self.qe[1,0]) + self.l4_y*np.sin(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])) + self.m3*(self.l3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]))*(self.L2_y*np.cos(self.qe[1,0]) + self.l3_y*np.sin(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.sin(self.qe[1,0])) - (self.l2_y**2*self.m2*np.sin(2*self.qe[1,0]))/2) - self.dqe[3,0]*np.sin(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])*(self.Ixx4*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]) - self.Izz4*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]) + self.l4_y**2*self.m4*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]) + self.L3_y*self.l4_y*self.m4*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*self.l4_y*self.m4*np.cos(self.qe[1,0]) - self.L2_y*self.l4_y*self.m4*np.sin(self.qe[1,0])))

        P12e = (-self.dqe[0,0]*((self.Ixx3*np.sin(2*self.qe[1,0] + 2*self.qe[2,0]))/2 - (self.Izz3*np.sin(2*self.qe[1,0] + 2*self.qe[2,0]))/2 - (self.Ixx2*np.sin(2*self.qe[1,0]))/2 + (self.Izz2*np.sin(2*self.qe[1,0]))/2 + (self.Ixx4*np.sin(2*self.qe[1,0] + 2*self.qe[2,0] + 2*self.qe[3,0]))/2 - (self.Izz4*np.sin(2*self.qe[1,0] + 2*self.qe[2,0] + 2*self.qe[3,0]))/2 + self.m4*(self.L3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]) + self.l4_y*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]))*(self.L3_y*np.sin(self.qe[1,0] + self.qe[2,0]) + self.L2_y*np.cos(self.qe[1,0]) + self.L2_x*np.sin(self.qe[1,0]) + self.l4_y*np.sin(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])) + self.m3*(self.l3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]))*(self.L2_y*np.cos(self.qe[1,0]) + self.l3_y*np.sin(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.sin(self.qe[1,0])) - (self.l2_y**2*self.m2*np.sin(2*self.qe[1,0]))/2))

        P13e = (-self.dqe[0,0]*((self.Ixx3*np.sin(2*self.qe[1,0] + 2*self.qe[2,0]))/2 - (self.Izz3*np.sin(2*self.qe[1,0] + 2*self.qe[2,0]))/2 + (self.Ixx4*np.sin(2*self.qe[1,0] + 2*self.qe[2,0] + 2*self.qe[3,0]))/2 - (self.Izz4*np.sin(2*self.qe[1,0] + 2*self.qe[2,0] + 2*self.qe[3,0]))/2 + self.m4*(self.L3_y*np.sin(self.qe[1,0] + self.qe[2,0]) + self.l4_y*np.sin(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]))*(self.L3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]) + self.l4_y*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])) + self.l3_y*self.m3*np.sin(self.qe[1,0] + self.qe[2,0])*(self.l3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]))))

        P14e = (-self.dqe[0,0]*np.sin(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])*(self.Ixx4*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]) - self.Izz4*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]) + self.l4_y**2*self.m4*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]) + self.L3_y*self.l4_y*self.m4*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*self.l4_y*self.m4*np.cos(self.qe[1,0]) - self.L2_y*self.l4_y*self.m4*np.sin(self.qe[1,0])))

        P21e = -P12e

        P22e = (self.Keq + self.dqe[2,0]*(self.L2_y*self.l4_y*self.m4*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.L3_y*self.m4*np.cos(self.qe[2,0]) - self.L2_x*self.l4_y*self.m4*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L2_x*self.L3_y*self.m4*np.sin(self.qe[2,0]) + self.L2_y*self.l3_y*self.m3*np.cos(self.qe[2,0]) - self.L2_x*self.l3_y*self.m3*np.sin(self.qe[2,0])) - self.l4_y*self.m4*self.dqe[3,0]*(self.L2_x*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L2_y*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L3_y*np.sin(self.qe[3,0])))

        P23e = (self.L2_y*self.l4_y*self.m4*self.dqe[1,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.l4_y*self.m4*self.dqe[2,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.l4_y*self.m4*self.dqe[3,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.L3_y*self.m4*self.dqe[1,0]*np.cos(self.qe[2,0]) + self.L2_y*self.L3_y*self.m4*self.dqe[2,0]*np.cos(self.qe[2,0]) - self.L2_x*self.l4_y*self.m4*self.dqe[1,0]*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L2_x*self.l4_y*self.m4*self.dqe[2,0]*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L2_x*self.l4_y*self.m4*self.dqe[3,0]*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L2_x*self.L3_y*self.m4*self.dqe[1,0]*np.sin(self.qe[2,0]) - self.L2_x*self.L3_y*self.m4*self.dqe[2,0]*np.sin(self.qe[2,0]) + self.L2_y*self.l3_y*self.m3*self.dqe[1,0]*np.cos(self.qe[2,0]) + self.L2_y*self.l3_y*self.m3*self.dqe[2,0]*np.cos(self.qe[2,0]) - self.L2_x*self.l3_y*self.m3*self.dqe[1,0]*np.sin(self.qe[2,0]) - self.L2_x*self.l3_y*self.m3*self.dqe[2,0]*np.sin(self.qe[2,0]) - self.L3_y*self.l4_y*self.m4*self.dqe[3,0]*np.sin(self.qe[3,0]))

        P24e = (-self.l4_y*self.m4*(self.L2_x*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L2_y*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L3_y*np.sin(self.qe[3,0]))*(self.dqe[1,0] + self.dqe[2,0] + self.dqe[3,0]))

        P31e = -P13e

        P32e = (self.L2_x*self.l4_y*self.m4*self.dqe[1,0]*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L2_y*self.L3_y*self.m4*self.dqe[1,0]*np.cos(self.qe[2,0]) - self.L2_y*self.l4_y*self.m4*self.dqe[1,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_x*self.L3_y*self.m4*self.dqe[1,0]*np.sin(self.qe[2,0]) - self.L2_y*self.l3_y*self.m3*self.dqe[1,0]*np.cos(self.qe[2,0]) + self.L2_x*self.l3_y*self.m3*self.dqe[1,0]*np.sin(self.qe[2,0]) - self.L3_y*self.l4_y*self.m4*self.dqe[3,0]*np.sin(self.qe[3,0]))

        P33e = (self.Keq + -self.L3_y*self.l4_y*self.m4*self.dqe[3,0]*np.sin(self.qe[3,0]))

        P34e = (-self.L3_y*self.l4_y*self.m4*np.sin(self.qe[3,0])*(self.dqe[1,0] + self.dqe[2,0] + self.dqe[3,0]))

        P41e = -P14e

        P42e = (self.l4_y*self.m4*(self.L3_y*self.dqe[1,0]*np.sin(self.qe[3,0]) + self.L3_y*self.dqe[2,0]*np.sin(self.qe[3,0]) - self.L2_y*self.dqe[1,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_x*self.dqe[1,0]*np.sin(self.qe[2,0] + self.qe[3,0])))

        P43e = (self.L3_y*self.l4_y*self.m4*np.sin(self.qe[3,0])*(self.dqe[1,0] + self.dqe[2,0]))

        P44e = (self.Keq)

        Pe  = self.U_lin*np.array([[P11e, P12e, P13e, P14e], 
                            [P21e, P22e, P23e, P24e], 
                            [P31e, P32e, P33e, P34e],
                            [P41e, P42e, P43e, P44e]])

        ##########

        D2e = (self.g*self.l2_y*self.m2*np.sin(self.qe[1,0]) - self.g*self.m4*(self.L3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0]) + self.l4_y*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])) - self.g*self.m3*(self.l3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.L2_x*np.cos(self.qe[1,0]) - self.L2_y*np.sin(self.qe[1,0])))
        D3e = (- self.g*self.m4*(self.L3_y*np.cos(self.qe[1,0] + self.qe[2,0]) + self.l4_y*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])) - self.g*self.l3_y*self.m3*np.cos(self.qe[1,0] + self.qe[2,0]))
        D4e = (-self.g*self.l4_y*self.m4*np.cos(self.qe[1,0] + self.qe[2,0] + self.qe[3,0]))

        Ge = self.U_lin*np.array([[0], [D2e], [D3e], [D4e]])
        
        # LEY DE CONTROL 
        self.u =(Me@self.ddqd + Pe@dqr + Ge - self.Kd@(self.dqe - dqr) - self.K1@z1)
        u2=(Me[1,:]@self.ddqd + Pe[1,:]@dqr + Ge[1,:] - self.Kd_u2[1,:]@(self.dqe - dqr) - self.K1_u2[1,:]@z1)
        self.u[1,0]=u2
        # Limitador de corriente:
        if self.u[0,0] >= self.u_lim:
            self.u[0,0] = self.u_lim
        elif self.u[0,0] <= -self.u_lim:
            self.u[0,0] = -self.u_lim

        if self.u[1,0] >= self.u_lim:
            self.u[1,0] = self.u_lim
        elif self.u[1,0] <= -self.u_lim:
            self.u[1,0] = -self.u_lim

        if self.u[2,0] >= self.u_lim:
            self.u[2,0] = self.u_lim
        elif self.u[2,0] <= -self.u_lim:
            self.u[2,0] = -self.u_lim

        if self.u[3,0] >= self.u_lim:
            self.u[3,0] = self.u_lim
        elif self.u[3,0] <= -self.u_lim:
            self.u[3,0] = -self.u_lim
        
        # Separando la señal de control
        u_sig1=self.u[0,0]
        u_sig2=self.u[1,0]
        u_sig3=self.u[2,0]
        u_sig4=self.u[3,0]

        self.u_sig = [u_sig1, u_sig2, u_sig3, u_sig4]

        


def main():
    rospy.init_node('bs_control')
    freq = 100
    rate = rospy.Rate(freq) #10 Hz
    control=controler(freq)

    while not rospy.is_shutdown():
        control.en_control()
        control.publish_present_u()
        control.publish_present_dqe()
        control.csv_getData()
        # Q=np.round(np.rad2deg(control.q),4)
        U=np.round(control.u_sig,4)
        #print("U1:",control.u[0],"\n")
        #print("\n ------- \n")
        #print("U2:",control.u[1],"\n")
        #rospy.loginfo(str(Q))
        rospy.loginfo(str(U))
        rate.sleep()

if __name__ == "__main__":
    main()