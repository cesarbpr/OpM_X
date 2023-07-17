#!/usr/bin/env python3
import rospy
import numpy as np
import csv
import os
from std_msgs.msg import Float64MultiArray

class controler:
    # PARÁMETROS DE SINTONIZACIÓN

    # Funciona para referencia
    Lambda=3.5
    ld = 15
    Kd =0.5
    Gamma = 0.001

    '''
    # Funciona para seguimiento
    Lambda=3.5
    ld = 30
    Kd =1.3
    Gamma = 0.01
    '''

    s_lim=0.006
    u_lim=0.672
    # PARÁMETROS DE CONTROL
    I = np.eye(4,4)
    Ld = ld*I
    # DATOS DEL SUB-SISTEMA MECANICO
    # Masas
    m1 = 0.105
    m2 = 0.142
    m3 = 0.135
    m4 = 0.236
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
    Jeq=0.0037
    #Gravedad y relacion de trnasmision
    #   n=353.5
    n=353.5
    g=-9.81
    #Constantes de friccion motores:
    # Bm=0.0001 Bg=0.01 
    # Beq=(n**2)*Bm+Bg
    Beq=0.00012
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
    # PARAMETROS DEL MODELO DINAMICO:
    ae1=l2_y*m2
    ae2=l2_y**2*m2
    ae3=m3
    ae4=l3_y*m3
    ae5=l3_y**2*m3
    ae6=m4
    ae7=l4_y*m4
    ae8=l4_y**2*m4
    ae9=Izz1
    ae10=Iyy2
    ae11=(Ixx2+Izz2)
    ae12=(Izz2-Ixx2)
    ae13=(Ixx2-Izz2)
    ae14=Iyy3
    ae15=(Ixx3+Izz3)
    ae16=(Izz3-Ixx3)
    ae17=(Ixx3-Izz3)
    ae18=Iyy4
    ae19=(Ixx4+Izz4)
    ae20=(Izz4-Ixx4)
    ae21=(Ixx4-Izz4)
    ae22=Jeq
    ae23=Keq
    ae=U_lin*np.array([[ae1],[ae2],[ae3],[ae4],[ae5],[ae6],[ae7],[ae8],[ae9],[ae10],[ae11],[ae12],[ae13],[ae14],[ae15],[ae16],[ae17],[ae18],[ae19],[ae20],[ae11],[ae22],[ae23]])
    # Iniciaalizar parámetros

    # CONDICIONES INICIALES
    # VALORES INICIALES
    q = np.array([[np.deg2rad(0)], [np.deg2rad(-20)],[np.deg2rad(0)],[np.deg2rad(-70)]], dtype=np.float64)
    dq = np.array([[0], [0],[0], [0]], dtype=np.float64)
    qe= np.array([[np.deg2rad(0)], [np.deg2rad(-20)],[np.deg2rad(0)],[np.deg2rad(-70)]], dtype=np.float64) 
    dqe=np.array([[0], [0],[0], [0]], dtype=np.float64)
    dqd = np.array([[0], [0],[0], [0]], dtype=np.float64)
    ddqd =  np.array([[0], [0],[0], [0]], dtype=np.float64)
    qd=np.array([[0], [0],[0], [0]], dtype=np.float64)
    qtilde = np.array([[0], [0],[0], [0]], dtype=np.float64)
    dqr = np.array([[np.deg2rad(0)], [np.deg2rad(0)],[np.deg2rad(0)],[np.deg2rad(0)]], dtype=np.float64)
    ddqr =  np.array([[0], [0],[0], [0]], dtype=np.float64)
    s =  np.array([[0], [0],[0], [0]], dtype=np.float64)
    dqtilde =  np.array([[0], [0],[0], [0]], dtype=np.float64)
    u_sig = np.array([0,0,0,0], dtype=np.float64)


    def __init__(self,sample_time):
        #pub_topic_u="u"
        #sub_topic_q_des="q_des"
        #sub_topic_q_real="q_real"
        pub_topic_u="u"
        sub_topic_q_des="q_des"
        sub_topic_q_real="pos_present_value"  

        #Timepo de muestreo
        self.T=1/sample_time

        self.pub_u=rospy.Publisher(pub_topic_u,Float64MultiArray,queue_size=10)
        self.qd_suscriber=rospy.Subscriber(sub_topic_q_des,Float64MultiArray,self.cb_qdes_in)
        self.qreal_suscriber=rospy.Subscriber(sub_topic_q_real,Float64MultiArray,self.cb_qreal_in)
    
    # Para la simplemenación
    def cb_qreal_in(self,q_real):
        q1=q_real.data[0]
        q2=q_real.data[1]
        q3=q_real.data[2]
        q4=q_real.data[3]
        self.q = np.array([[q1], [q2], [q3], [q4]])

    def publish_present_u(self):
        pub_array = Float64MultiArray()
        pub_array.data = [0,0,0,0]
        pub_array.data[0] = self.u[0,0]
        pub_array.data[1] = self.u[1,0]
        pub_array.data[2] = self.u[2,0]
        pub_array.data[3] = self.u[3,0]
        self.pub_u.publish(pub_array)

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
        file_path = os.path.join(current_dir, 'Dato_ac_simu_4GdL.csv')
        with open(file_path,'a') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([current_time] + array_q)

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

        
        # MATRIZ Y DE PARÁMETROS NO LINEALES
        sig1=(2*self.qe[1,0] + 2*self.qe[2,0] + 2*self.qe[3,0])
        sig2=(2*self.qe[1,0] + 2*self.qe[2,0] + self.qe[3,0])
        sig3=(2*self.qe[1,0] + self.qe[2,0] + self.qe[3,0])
        sig4=(self.qe[1,0] + self.qe[2,0] + self.qe[3,0])
        sig5=(2*self.qe[1,0] + 2*self.qe[2,0])
        sig6=(2*self.qe[1,0] + self.qe[2,0])
        # Tau1=
        Y12=self.ddqr[0,0]/2 - self.ddqr[0,0]*np.cos(2*self.qe[1,0])/2 + self.dqe[0,0]*self.dqe[1,0]*np.sin(2*self.qe[1,0])
        Y13=(self.L2_x**2*self.ddqr[0,0])/2 + (self.L2_x**2*self.ddqr[0,0]*np.cos(2*self.qe[1,0]))/2 - self.L2_x**2*self.dqe[0,0]*self.dqe[1,0]*np.sin(2*self.qe[1,0])+ (self.L2_y**2*self.ddqr[0,0])/2 - (self.L2_y**2*self.ddqr[0,0]*np.cos(2*self.qe[1,0]))/2 + self.L2_y**2*self.dqe[0,0]*self.dqe[1,0]*np.sin(2*self.qe[1,0]) - self.L2_x*self.L2_y*self.ddqr[0,0]*np.sin(2*self.qe[1,0]) - 2*self.L2_x*self.L2_y*self.dqe[0,0]*self.dqe[1,0]*np.cos(2*self.qe[1,0])  
        Y14=self.L2_x*self.ddqr[0,0]*np.cos(self.qe[2,0]) - self.L2_y*self.ddqr[0,0]*np.sin(sig6) + self.L2_y*self.ddqr[0,0]*np.sin(self.qe[2,0]) + self.L2_x*self.ddqr[0,0]*np.cos(sig6) + self.L2_y*self.dqe[0,0]*self.dqe[2,0]*np.cos(self.qe[2,0]) - self.L2_x*self.dqe[0,0]*self.dqe[2,0]*np.sin(self.qe[2,0]) - 2*self.L2_y*self.dqe[0,0]*self.dqe[1,0]*np.cos(sig6) - self.L2_y*self.dqe[0,0]*self.dqe[2,0]*np.cos(sig6) - 2*self.L2_x*self.dqe[0,0]*self.dqe[1,0]*np.sin(sig6) - self.L2_x*self.dqe[0,0]*self.dqe[2,0]*np.sin(sig6) 
        Y15=((self.ddqr[0,0])/2 + (self.ddqr[0,0]*np.cos(sig5))/2 - self.dqe[0,0]*self.dqe[1,0]*np.sin(sig5) - self.dqe[0,0]*self.dqe[2,0]*np.sin(sig5)) 
        Y16=(self.L2_x**2*self.ddqr[0,0])/2 + (self.L2_x**2*self.ddqr[0,0]*np.cos(2*self.qe[1,0]))/2 - self.L2_x**2*self.dqe[0,0]*self.dqe[1,0]*np.sin(2*self.qe[1,0]) + (self.L2_y**2*self.ddqr[0,0])/2 - (self.L2_y**2*self.ddqr[0,0]*np.cos(2*self.qe[1,0]))/2 + self.L2_y**2*self.dqe[0,0]*self.dqe[1,0]*np.sin(2*self.qe[1,0]) + (self.L3_y**2*self.ddqr[0,0])/2 + (self.L3_y**2*self.ddqr[0,0]*np.cos(sig5))/2 - self.L3_y**2*self.dqe[0,0]*self.dqe[1,0]*np.sin(sig5) - self.L3_y**2*self.dqe[0,0]*self.dqe[2,0]*np.sin(sig5) + self.L2_x*self.L3_y*self.ddqr[0,0]*np.cos(self.qe[2,0]) + self.L2_y*self.L3_y*self.ddqr[0,0]*np.sin(self.qe[2,0]) + self.L2_x*self.L3_y*self.ddqr[0,0]*np.cos(sig6) - self.L2_y*self.L3_y*self.ddqr[0,0]*np.sin(sig6) + self.L2_y*self.L3_y*self.dqe[0,0]*self.dqe[2,0]*np.cos(self.qe[2,0]) - self.L2_x*self.L3_y*self.dqe[0,0]*self.dqe[2,0]*np.sin(self.qe[2,0]) - 2*self.L2_y*self.L3_y*self.dqe[0,0]*self.dqe[1,0]*np.cos(sig6) - self.L2_y*self.L3_y*self.dqe[0,0]*self.dqe[2,0]*np.cos(sig6) - 2*self.L2_x*self.L3_y*self.dqe[0,0]*self.dqe[1,0]*np.sin(sig6) - self.L2_x*self.L3_y*self.dqe[0,0]*self.dqe[2,0]*np.sin(sig6) - self.L2_x*self.L2_y*self.ddqr[0,0]*np.sin(2*self.qe[1,0]) - 2*self.L2_x*self.L2_y*self.dqe[0,0]*self.dqe[1,0]*np.cos(2*self.qe[1,0]) 
        Y17=(self.L2_x*self.ddqr[0,0]*np.cos(sig3) - self.L2_y*self.ddqr[0,0]*np.sin(sig3) + self.L2_x*self.ddqr[0,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.ddqr[0,0]*np.sin(self.qe[2,0] + self.qe[3,0]) + self.L3_y*self.ddqr[0,0]*np.cos(self.qe[3,0]) + self.L3_y*self.ddqr[0,0]*np.cos(sig2) + self.L2_y*self.dqe[0,0]*self.dqe[2,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.dqe[0,0]*self.dqe[3,0])*np.cos(self.qe[2,0] + self.qe[3,0]) - self.L2_x*self.dqe[0,0]*self.dqe[2,0]*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L2_x*self.dqe[0,0]*self.dqe[3,0]*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L3_y*self.dqe[0,0]*self.dqe[3,0]*np.sin(self.qe[3,0]) - 2*self.L3_y*self.dqe[0,0]*self.dqe[1,0]*np.sin(sig2) - 2*self.L3_y*self.dqe[0,0]*self.dqe[2,0]*np.sin(sig2) - self.L3_y*self.dqe[0,0]*self.dqe[3,0]*np.sin(sig2) - 2*self.L2_y*self.dqe[0,0]*self.dqe[1,0]*np.cos(sig3) - self.L2_y*self.dqe[0,0]*self.dqe[2,0]*np.cos(sig3) - self.L2_y*self.dqe[0,0]*self.dqe[3,0]*np.cos(sig3) - 2*self.L2_x*self.dqe[0,0]*self.dqe[1,0]*np.sin(sig3) - self.L2_x*self.dqe[0,0]*self.dqe[2,0]*np.sin(sig3) - self.L2_x*self.dqe[0,0]*self.dqe[3,0]*np.sin(sig3) 
        Y18=((self.ddqr[0,0])/2 - self.dqe[0,0]*self.dqe[1,0]*np.sin(sig1) + (self.ddqr[0,0]*np.cos(sig1))/2 - self.dqe[0,0]*self.dqe[2,0]*np.sin(sig1) - self.dqe[0,0]*self.dqe[3,0])*np.sin(sig1) 
        Y19=self.ddqr[0,0] 
        Y111=self.ddqr[0,0]/2 
        Y112=self.ddqr[0,0]*np.cos(2*self.qe[1,0])/2 
        Y113=self.dqe[0,0]*self.dqe[1,0]*np.sin(2*self.qe[1,0]) 
        Y115=self.ddqr[0,0]/2 
        Y116=self.dqe[0,0]*self.dqe[1,0]*np.sin(sig5) + self.dqe[0,0]*self.dqe[2,0]*np.sin(sig5) 
        Y117=self.ddqr[0,0]*np.cos(sig5)/2 
        Y119=self.ddqr[0,0]/2 
        Y120=(self.dqe[0,0]*self.dqe[1,0]*np.sin(sig1) + self.dqe[0,0]*self.dqe[2,0]*np.sin(sig1) + self.dqe[0,0]*self.dqe[3,0])*np.sin(sig1) 
        Y121=self.ddqr[0,0]*np.cos(sig1)/2 
        Y122=self.ddqr[0,0] 
        Y123=self.dqe[0,0] 
        # Tau2=
        Y21=self.g*np.sin(self.qe[1,0]) 
        Y22=(self.ddqr[1,0] - (self.dqe[0,0]**2*np.sin(2*self.qe[1,0]))/2) 
        Y23=(self.L2_x**2*self.ddqr[1,0] + (self.L2_x**2*self.dqe[0,0]**2*np.sin(2*self.qe[1,0]))/2 + self.L2_y**2*self.ddqr[1,0] - (self.L2_y**2*self.dqe[0,0]**2*np.sin(2*self.qe[1,0]))/2 + self.L2_x*self.L2_y*self.dqe[0,0]**2*np.cos(2*self.qe[1,0]) - self.L2_x*self.g*np.cos(self.qe[1,0]) + self.L2_y*self.g*np.sin(self.qe[1,0])) 
        Y24=(self.L2_y*self.dqe[2,0]**2*np.cos(self.qe[2,0]) - self.g*np.cos(self.qe[1,0] + self.qe[2,0]) - self.L2_x*self.dqe[2,0]**2*np.sin(self.qe[2,0]) + self.L2_y*self.dqe[0,0]**2*np.cos(sig6) + self.L2_x*self.dqe[0,0]**2*np.sin(sig6) + 2*self.L2_x*self.ddqr[1,0]*np.cos(self.qe[2,0]) + self.L2_x*self.ddqr[2,0]*np.cos(self.qe[2,0]) + 2*self.L2_y*self.ddqr[1,0]*np.sin(self.qe[2,0]) + self.L2_y*self.ddqr[2,0]*np.sin(self.qe[2,0]) + 2*self.L2_y*self.dqe[1,0]*self.dqe[2,0]*np.cos(self.qe[2,0]) - 2*self.L2_x*self.dqe[1,0]*self.dqe[2,0]*np.sin(self.qe[2,0])) 
        Y25=(self.ddqr[1,0] + self.ddqr[2,0] + (self.dqe[0,0]**2*np.sin(sig5))/2) 
        Y26=(self.L2_x**2*self.ddqr[1,0] + (self.L2_x**2*self.dqe[0,0]**2*np.sin(2*self.qe[1,0]))/2 + self.L2_y**2*self.ddqr[1,0] + self.L3_y**2*self.ddqr[1,0] + self.L3_y**2*self.ddqr[2,0] + (self.L3_y**2*self.dqe[0,0]**2*np.sin(sig5))/2 - self.L3_y*self.g*np.cos(self.qe[1,0] + self.qe[2,0]) - self.L2_x*self.g*np.cos(self.qe[1,0]) + self.L2_y*self.g*np.sin(self.qe[1,0])  - (self.L2_y**2*self.dqe[0,0]**2*np.sin(2*self.qe[1,0]))/2 + self.L2_y*self.L3_y*self.dqe[2,0]**2*np.cos(self.qe[2,0]) - self.L2_x*self.L3_y*self.dqe[2,0]**2*np.sin(self.qe[2,0]) + self.L2_y*self.L3_y*self.dqe[0,0]**2*np.cos(sig6) + self.L2_x*self.L3_y*self.dqe[0,0]**2*np.sin(sig6) + self.L2_x*self.L2_y*self.dqe[0,0]**2*np.cos(2*self.qe[1,0]) + 2*self.L2_x*self.L3_y*self.ddqr[1,0]*np.cos(self.qe[2,0]) + self.L2_x*self.L3_y*self.ddqr[2,0]*np.cos(self.qe[2,0]) + 2*self.L2_y*self.L3_y*self.ddqr[1,0]*np.sin(self.qe[2,0]) + self.L2_y*self.L3_y*self.ddqr[2,0]*np.sin(self.qe[2,0]) + 2*self.L2_y*self.L3_y*self.dqe[1,0]*self.dqe[2,0]*np.cos(self.qe[2,0]) - 2*self.L2_x*self.L3_y*self.dqe[1,0]*self.dqe[2,0]*np.sin(self.qe[2,0])) 
        Y27=(self.L3_y*self.dqe[0,0]**2*np.sin(sig2) - self.L2_x*self.dqe[2,0]**2*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L2_x*self.dqe[3,0])**2*np.sin(self.qe[2,0] + self.qe[3,0]) - self.L3_y*self.dqe[3,0]**2*np.sin(self.qe[3,0]) + 2*self.L2_x*self.ddqr[1,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_x*self.ddqr[2,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_x*self.ddqr[3,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + 2*self.L2_y*self.ddqr[1,0]*np.sin(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.ddqr[2,0]*np.sin(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.ddqr[3,0]*np.sin(self.qe[2,0] + self.qe[3,0]) + 2*self.L3_y*self.ddqr[1,0]*np.cos(self.qe[3,0]) + 2*self.L3_y*self.ddqr[2,0]*np.cos(self.qe[3,0]) + self.L3_y*self.ddqr[3,0]*np.cos(self.qe[3,0])+ self.L2_y*self.dqe[0,0]**2*np.cos(sig3) + self.L2_x*self.dqe[0,0]**2*np.sin(sig3) + self.L2_y*self.dqe[2,0]**2*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.dqe[3,0]**2*np.cos(self.qe[2,0] + self.qe[3,0]) + 2*self.L2_y*self.dqe[1,0]*self.dqe[2,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + 2*self.L2_y*self.dqe[1,0]*self.dqe[3,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + 2*self.L2_y*self.dqe[2,0]*self.dqe[3,0]*np.cos(self.qe[2,0] + self.qe[3,0]) - 2*self.L2_x*self.dqe[1,0]*self.dqe[2,0]*np.sin(self.qe[2,0] + self.qe[3,0]) - 2*self.L2_x*self.dqe[1,0]*self.dqe[3,0]*np.sin(self.qe[2,0] + self.qe[3,0]) - 2*self.L2_x*self.dqe[2,0]*self.dqe[3,0]*np.sin(self.qe[2,0] + self.qe[3,0]) - 2*self.L3_y*self.dqe[1,0]*self.dqe[3,0]*np.sin(self.qe[3,0]) - 2*self.L3_y*self.dqe[2,0]*self.dqe[3,0]*np.sin(self.qe[3,0]) - self.g*np.cos(sig4) 
        Y28=(self.ddqr[1,0] + self.ddqr[2,0] + self.ddqr[3,0] + (self.dqe[0,0]**2*np.sin(sig1))/2) 
        Y210=self.ddqr[1,0] 
        Y212=self.dqe[0,0]**2*np.sin(2*self.qe[1,0])/2 
        Y214=(self.ddqr[1,0] + self.ddqr[2,0]) 
        Y217=self.dqe[0,0]**2*np.sin(sig5)/2 
        Y218=(self.ddqr[1,0] + self.ddqr[2,0] + self.ddqr[3,0]) 
        Y221=self.dqe[0,0]**2*np.sin(sig1)/2 
        Y222=self.ddqr[1,0] 
        Y223=self.dqe[1,0] 
        # Tau3=
        Y34=(self.L2_x*self.dqe[1,0]**2*np.sin(self.qe[2,0]) - self.g*np.cos(self.qe[1,0] + self.qe[2,0]) - self.L2_y*self.dqe[1,0]**2*np.cos(self.qe[2,0]) + self.L2_x*self.ddqr[1,0]*np.cos(self.qe[2,0]) + self.L2_y*self.ddqr[1,0]*np.sin(self.qe[2,0]) + self.L2_x*self.dqe[0,0]**2*np.sin(self.qe[1,0] + self.qe[2,0])*np.cos(self.qe[1,0]) - self.L2_y*self.dqe[0,0]**2*np.sin(self.qe[1,0] + self.qe[2,0])*np.sin(self.qe[1,0])) 
        Y35=(self.ddqr[1,0] + self.ddqr[2,0] + self.dqe[0,0]**2*np.cos(self.qe[1,0] + self.qe[2,0])*np.sin(self.qe[1,0] + self.qe[2,0])) 
        Y36=(self.L3_y**2*self.ddqr[1,0] + self.L3_y**2*self.ddqr[2,0] + self.L3_y**2*self.dqe[0,0]**2*np.cos(self.qe[1,0] + self.qe[2,0])*np.sin(self.qe[1,0] + self.qe[2,0]) - self.L3_y*self.g*np.cos(self.qe[1,0] + self.qe[2,0]) - self.L2_y*self.L3_y*self.dqe[1,0]**2*np.cos(self.qe[2,0]) + self.L2_x*self.L3_y*self.dqe[1,0]**2*np.sin(self.qe[2,0]) + self.L2_x*self.L3_y*self.ddqr[1,0]*np.cos(self.qe[2,0]) + self.L2_y*self.L3_y*self.ddqr[1,0]*np.sin(self.qe[2,0]) + self.L2_x*self.L3_y*self.dqe[0,0]**2*np.sin(self.qe[1,0] + self.qe[2,0])*np.cos(self.qe[1,0]) - self.L2_y*self.L3_y*self.dqe[0,0]**2*np.sin(self.qe[1,0] + self.qe[2,0])*np.sin(self.qe[1,0])) 
        Y37=(self.L2_x*self.dqe[1,0]**2*np.sin(self.qe[2,0] + self.qe[3,0]) - self.g*np.cos(sig4) - self.L3_y*self.dqe[3,0])**2*np.sin(self.qe[3,0]) + self.L2_x*self.ddqr[1,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.ddqr[1,0]*np.sin(self.qe[2,0] + self.qe[3,0]) + 2*self.L3_y*self.ddqr[1,0]*np.cos(self.qe[3,0]) + 2*self.L3_y*self.ddqr[2,0]*np.cos(self.qe[3,0])+ self.L3_y*self.ddqr[3,0]*np.cos(self.qe[3,0]) - self.L2_y*self.dqe[1,0]**2*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L3_y*self.dqe[0,0]**2*np.cos(sig4)*np.sin(self.qe[1,0] + self.qe[2,0]) + self.L3_y*self.dqe[0,0]**2*np.sin(sig4)*np.cos(self.qe[1,0] + self.qe[2,0]) - 2*self.L3_y*self.dqe[1,0]*self.dqe[3,0]*np.sin(self.qe[3,0]) - 2*self.L3_y*self.dqe[2,0]*self.dqe[3,0]*np.sin(self.qe[3,0]) + self.L2_x*self.dqe[0,0]**2*np.sin(sig4)*np.cos(self.qe[1,0]) - self.L2_y*self.dqe[0,0]**2*np.sin(sig4)*np.sin(self.qe[1,0]) 
        Y38=(self.ddqr[1,0] + self.ddqr[2,0] + self.ddqr[3,0] + self.dqe[0,0]**2*np.cos(sig4)*np.sin(sig4)) 
        Y314=(self.ddqr[1,0] + self.ddqr[2,0]) 
        Y317=self.dqe[0,0]**2*np.sin(sig5)/2 
        Y318=(self.ddqr[1,0] + self.ddqr[2,0] + self.ddqr[3,0]) 
        Y321=self.dqe[0,0]**2*np.sin(sig1)/2 
        Y322=self.ddqr[2,0] 
        Y323=self.dqe[2,0]
        # Tau4=0 
        Y47=(self.L2_x*self.dqe[1,0]**2*np.sin(self.qe[2,0] + self.qe[3,0]) + self.L3_y*self.dqe[1,0]**2*np.sin(self.qe[3,0]) + self.L3_y*self.dqe[2,0]**2*np.sin(self.qe[3,0]) + self.L2_x*self.ddqr[1,0]*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L2_y*self.ddqr[1,0]*np.sin(self.qe[2,0] + self.qe[3,0]) + self.L3_y*self.ddqr[1,0]*np.cos(self.qe[3,0]) + self.L3_y*self.ddqr[2,0]*np.cos(self.qe[3,0]) - self.L2_y*self.dqe[1,0]**2*np.cos(self.qe[2,0] + self.qe[3,0]) + self.L3_y*self.dqe[0,0]**2*np.sin(sig4)*np.cos(self.qe[1,0] + self.qe[2,0]) + 2*self.L3_y*self.dqe[1,0]*self.dqe[2,0]*np.sin(self.qe[3,0]) + self.L2_x*self.dqe[0,0]**2*np.sin(sig4)*np.cos(self.qe[1,0]) - self.L2_y*self.dqe[0,0]**2*np.sin(sig4)*np.sin(self.qe[1,0]) - self.g*np.cos(sig4)) 
        Y48=(self.ddqr[1,0] + self.ddqr[2,0] + self.ddqr[3,0] + self.dqe[0,0]**2*np.cos(sig4)*np.sin(sig4)) 
        Y418=(self.ddqr[1,0] + self.ddqr[2,0] + self.ddqr[3,0]) 
        Y421=self.dqe[0,0]**2*np.cos(sig4)*np.sin(sig4) 
        Y422=self.ddqr[3,0] 
        Y423=self.dqe[3,0] 

        Y = np.array([[0  , Y12, Y13, Y14, Y15, Y16, Y17, Y18, Y19, 0   , Y111, Y112, Y113, 0   , Y115, Y116, Y117, 0   , Y119, Y120, Y121, Y122, Y123],
                    [Y21, Y22, Y23, Y24, Y25, Y26, Y27, Y28, 0  , Y210, 0   , Y212, 0   , Y214, 0   , 0   , Y217, Y218, 0   , 0   , Y221, Y222, Y223],
                    [0  , 0  , 0  , Y34, Y35, Y36, Y37, Y38, 0  , 0   , 0   , 0   , 0   , Y314, 0   , 0   , Y317, Y318, 0   , 0   , Y321, Y322, Y323],
                    [0  , 0  , 0  , 0  , 0  , 0  , Y47, Y48, 0  , 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , Y418, 0   , 0   , Y421, Y422, Y423]])
            
        # FUNCION DE DESLIZAMIENTO 
        self.s = self.dqtilde + self.Lambda*self.qtilde

        # ESTIMACION DE PARAMETROS
        Y_trans = np.transpose(Y)
        if (self.s[0,0]<self.s_lim) and (self.s[0,0]>-self.s_lim):
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
    rospy.init_node('ac_control')
    freq = 100
    rate = rospy.Rate(freq) #10 Hz
    control=controler(freq)

    while not rospy.is_shutdown():
        control.cb_control()
        control.publish_present_u()
        U=np.round(control.u_sig,4)
        #print("U1:",control.u[0],"\n")
        #print("\n ------- \n")
        #print("U2:",control.u[1],"\n")
        rospy.loginfo(str(U))
        rate.sleep()

if __name__ == "__main__":
    main()