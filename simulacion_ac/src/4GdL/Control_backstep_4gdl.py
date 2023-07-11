#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import pdb
import time
# CONTROL BACKSTEPPING PARA ROBOT DE 4GDL

# LAS ENTRADAS SON LOS PARAMETROS DE SINTONIZACION
k1=0.8
ld=16
kd=0.3
kk=1.5
# PARAMETROS DE CONTROL
I = np.eye(4,4)
K  = kk*I
Kd = kd*I
K1 = k1*I
Ld = ld*I
#tap=1
u_lim=10

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
u_pas=np.array([[0], [0], [0], [0]], dtype=np.float64)


# TIEMPO DE MUESTREO
T = 0.01
Tp = 5
A = 1.2
W = 2*np.pi*T/Tp   
nn = 1000

# VARIABLES PARA ALMACENAR DATOS
Q1=np.zeros((1,nn), dtype=np.float64)
Q2=np.zeros((1,nn), dtype=np.float64)
Q3=np.zeros((1,nn), dtype=np.float64)
Q4=np.zeros((1,nn), dtype=np.float64)
Qd1=np.zeros((1,nn), dtype=np.float64)
Qd2=np.zeros((1,nn), dtype=np.float64)
Qd3=np.zeros((1,nn), dtype=np.float64)
Qd4=np.zeros((1,nn), dtype=np.float64)
U1=np.zeros((1,nn), dtype=np.float64)
U2=np.zeros((1,nn), dtype=np.float64)
U3=np.zeros((1,nn), dtype=np.float64)
U4=np.zeros((1,nn), dtype=np.float64)
tr=2-1
execution_time_max=0

# LAZO DE CONTROL
for k in np.arange(0, nn):
    start_time = time.time()
    # REFERENCIAS DESEADAS Cuadradas
    if k >= 0 and k <= nn/4:
        qd[0,0] = np.deg2rad(0)
        qd[1,0] = np.deg2rad(-20)
        qd[2,0] = np.deg2rad(0)
        qd[3,0] = np.deg2rad(-70)
    elif k >= nn/4 and k <= nn/2:
        qd[0,0] = np.deg2rad(30)
        qd[1,0] = np.deg2rad(-30)
        qd[2,0] = np.deg2rad(50)
        qd[3,0] = np.deg2rad(-50)
    elif k >= nn/2 and k <= 3*nn/4:
        qd[0,0] = np.deg2rad(-20)
        qd[1,0] = np.deg2rad(30)
        qd[2,0] = np.deg2rad(-30)
        qd[3,0] = np.deg2rad(40)
    elif k >= 3*nn/4 and k <= nn:
        qd[0,0] = np.deg2rad(0)
        qd[1,0] = np.deg2rad(0)
        qd[2,0] = np.deg2rad(0)
        qd[3,0] = np.deg2rad(0)

    Qd1[0,k] = qd[0,0]
    Qd2[0,k] = qd[1,0]
    Qd3[0,k] = qd[2,0]
    Qd4[0,k] = qd[3,0]

    
    dqd[0] = 0
    dqd[1] = 0
    dqd[2] = 0
    dqd[3] = 0

    ddqd[0] = 0
    ddqd[1] = 0
    ddqd[2] = 0
    ddqd[3] = 0

    # ERROR TRACKING
    e = q - qd
    z1 = e

    dqr = dqd - K@z1

    # OBSERVADOR DE VELOCIDAD
    dqe = dqd + Ld@(q - qe)
    qe = qe + T*dqe
    
    # DINAMICA DEL MANIPULADOR
    # Matriz Ge inercias:
    M11e = (Jeq+Ixx3 + Ixx4 + Izz1 + Izz2 + m3*(l3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]))**2 - Ixx4*np.sin(qe[1,0] + qe[2,0] + qe[3,0])**2 + Izz4*np.sin(qe[1,0] + qe[2,0] + qe[3,0])**2 - Ixx3*np.sin(qe[1,0] + qe[2,0])**2 + Izz3*np.sin(qe[1,0] + qe[2,0])**2 + m4*(L3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]) + l4_y*np.cos(qe[1,0] + qe[2,0] + qe[3,0]))**2 + Ixx2*np.sin(qe[1,0])**2 - Izz2*np.sin(qe[1,0])**2 + l2_y**2*m2*np.sin(qe[1,0])**2)

    M22e = (Jeq+Iyy2 + Iyy3 + Iyy4 + L2_x**2*m3 + L2_x**2*m4 + L2_y**2*m3 + L2_y**2*m4 + L3_y**2*m4 + l2_y**2*m2 + l3_y**2*m3 + l4_y**2*m4 + 2*L2_x*l4_y*m4*np.cos(qe[2,0] + qe[3,0]) + 2*L2_x*L3_y*m4*np.cos(qe[2,0]) + 2*L2_y*l4_y*m4*np.sin(qe[2,0] + qe[3,0]) + 2*L2_y*L3_y*m4*np.sin(qe[2,0]) + 2*L2_x*l3_y*m3*np.cos(qe[2,0]) + 2*L3_y*l4_y*m4*np.cos(qe[3,0]) + 2*L2_y*l3_y*m3*np.sin(qe[2,0]))

    M23e = (Iyy3 + Iyy4 + L3_y**2*m4 + l3_y**2*m3 + l4_y**2*m4 + L2_x*l4_y*m4*np.cos(qe[2,0] + qe[3,0]) + L2_x*L3_y*m4*np.cos(qe[2,0]) + L2_y*l4_y*m4*np.sin(qe[2,0] + qe[3,0]) + L2_y*L3_y*m4*np.sin(qe[2,0]) + L2_x*l3_y*m3*np.cos(qe[2,0]) + 2*L3_y*l4_y*m4*np.cos(qe[3,0]) + L2_y*l3_y*m3*np.sin(qe[2,0]))

    M24e = (Iyy4 + l4_y*m4*(l4_y + L2_x*np.cos(qe[2,0] + qe[3,0]) + L2_y*np.sin(qe[2,0] + qe[3,0]) + L3_y*np.cos(qe[3,0])))

    M32e = (Iyy3 + Iyy4 + L3_y**2*m4 + l3_y**2*m3 + l4_y**2*m4 + L2_x*l4_y*m4*np.cos(qe[2,0] + qe[3,0]) + L2_x*L3_y*m4*np.cos(qe[2,0]) + L2_y*l4_y*m4*np.sin(qe[2,0] + qe[3,0]) + L2_y*L3_y*m4*np.sin(qe[2,0]) + L2_x*l3_y*m3*np.cos(qe[2,0]) + 2*L3_y*l4_y*m4*np.cos(qe[3,0]) + L2_y*l3_y*m3*np.sin(qe[2,0]))

    M33e =  (Jeq+Iyy3 + Iyy4 + m4*(L3_y**2 + 2*np.cos(qe[3,0])*L3_y*l4_y + l4_y**2) + l3_y**2*m3)

    M34e = (Iyy4 + m4*(l4_y**2 + L3_y*np.cos(qe[3,0])*l4_y))

    M42e= (Iyy4 + l4_y*m4*(l4_y + L2_x*np.cos(qe[2,0] + qe[3,0]) + L2_y*np.sin(qe[2,0] + qe[3,0]) + L3_y*np.cos(qe[3,0])))

    M43e=  (Iyy4 + m4*(l4_y**2 + L3_y*np.cos(qe[3,0])*l4_y))

    M44e=  (Jeq+ m4*l4_y**2 + Iyy4)

    Me = U_lin*np.array([[M11e  ,  0   ,     0   ,    0] ,
                          [0 ,   M22e  ,   M23e   ,M24e],
                          [0   , M32e  ,   M33e ,  M34e],
                          [0  ,  M42e   ,  M43e ,  M44e]])
    ############
    P11e = (Keq - dqe[2,0]*((Ixx3*np.sin(2*qe[1,0] + 2*qe[2,0]))/2 - (Izz3*np.sin(2*qe[1,0] + 2*qe[2,0]))/2 + (Ixx4*np.sin(2*qe[1,0] + 2*qe[2,0] + 2*qe[3,0]))/2 - (Izz4*np.sin(2*qe[1,0] + 2*qe[2,0] + 2*qe[3,0]))/2 + m4*(L3_y*np.sin(qe[1,0] + qe[2,0]) + l4_y*np.sin(qe[1,0] + qe[2,0] + qe[3,0]))*(L3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]) + l4_y*np.cos(qe[1,0] + qe[2,0] + qe[3,0])) + l3_y*m3*np.sin(qe[1,0] + qe[2,0])*(l3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]))) - dqe[1,0]*((Ixx3*np.sin(2*qe[1,0] + 2*qe[2,0]))/2 - (Izz3*np.sin(2*qe[1,0] + 2*qe[2,0]))/2 - (Ixx2*np.sin(2*qe[1,0]))/2 + (Izz2*np.sin(2*qe[1,0]))/2 + (Ixx4*np.sin(2*qe[1,0] + 2*qe[2,0] + 2*qe[3,0]))/2 - (Izz4*np.sin(2*qe[1,0] + 2*qe[2,0] + 2*qe[3,0]))/2 + m4*(L3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]) + l4_y*np.cos(qe[1,0] + qe[2,0] + qe[3,0]))*(L3_y*np.sin(qe[1,0] + qe[2,0]) + L2_y*np.cos(qe[1,0]) + L2_x*np.sin(qe[1,0]) + l4_y*np.sin(qe[1,0] + qe[2,0] + qe[3,0])) + m3*(l3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]))*(L2_y*np.cos(qe[1,0]) + l3_y*np.sin(qe[1,0] + qe[2,0]) + L2_x*np.sin(qe[1,0])) - (l2_y**2*m2*np.sin(2*qe[1,0]))/2) - dqe[3,0]*np.sin(qe[1,0] + qe[2,0] + qe[3,0])*(Ixx4*np.cos(qe[1,0] + qe[2,0] + qe[3,0]) - Izz4*np.cos(qe[1,0] + qe[2,0] + qe[3,0]) + l4_y**2*m4*np.cos(qe[1,0] + qe[2,0] + qe[3,0]) + L3_y*l4_y*m4*np.cos(qe[1,0] + qe[2,0]) + L2_x*l4_y*m4*np.cos(qe[1,0]) - L2_y*l4_y*m4*np.sin(qe[1,0])))

    P12e = (-dqe[0,0]*((Ixx3*np.sin(2*qe[1,0] + 2*qe[2,0]))/2 - (Izz3*np.sin(2*qe[1,0] + 2*qe[2,0]))/2 - (Ixx2*np.sin(2*qe[1,0]))/2 + (Izz2*np.sin(2*qe[1,0]))/2 + (Ixx4*np.sin(2*qe[1,0] + 2*qe[2,0] + 2*qe[3,0]))/2 - (Izz4*np.sin(2*qe[1,0] + 2*qe[2,0] + 2*qe[3,0]))/2 + m4*(L3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]) + l4_y*np.cos(qe[1,0] + qe[2,0] + qe[3,0]))*(L3_y*np.sin(qe[1,0] + qe[2,0]) + L2_y*np.cos(qe[1,0]) + L2_x*np.sin(qe[1,0]) + l4_y*np.sin(qe[1,0] + qe[2,0] + qe[3,0])) + m3*(l3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]))*(L2_y*np.cos(qe[1,0]) + l3_y*np.sin(qe[1,0] + qe[2,0]) + L2_x*np.sin(qe[1,0])) - (l2_y**2*m2*np.sin(2*qe[1,0]))/2))

    P13e = (-dqe[0,0]*((Ixx3*np.sin(2*qe[1,0] + 2*qe[2,0]))/2 - (Izz3*np.sin(2*qe[1,0] + 2*qe[2,0]))/2 + (Ixx4*np.sin(2*qe[1,0] + 2*qe[2,0] + 2*qe[3,0]))/2 - (Izz4*np.sin(2*qe[1,0] + 2*qe[2,0] + 2*qe[3,0]))/2 + m4*(L3_y*np.sin(qe[1,0] + qe[2,0]) + l4_y*np.sin(qe[1,0] + qe[2,0] + qe[3,0]))*(L3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]) + l4_y*np.cos(qe[1,0] + qe[2,0] + qe[3,0])) + l3_y*m3*np.sin(qe[1,0] + qe[2,0])*(l3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]))))

    P14e = (-dqe[0,0]*np.sin(qe[1,0] + qe[2,0] + qe[3,0])*(Ixx4*np.cos(qe[1,0] + qe[2,0] + qe[3,0]) - Izz4*np.cos(qe[1,0] + qe[2,0] + qe[3,0]) + l4_y**2*m4*np.cos(qe[1,0] + qe[2,0] + qe[3,0]) + L3_y*l4_y*m4*np.cos(qe[1,0] + qe[2,0]) + L2_x*l4_y*m4*np.cos(qe[1,0]) - L2_y*l4_y*m4*np.sin(qe[1,0])))

    P21e = -P12e

    P22e = (Keq + dqe[2,0]*(L2_y*l4_y*m4*np.cos(qe[2,0] + qe[3,0]) + L2_y*L3_y*m4*np.cos(qe[2,0]) - L2_x*l4_y*m4*np.sin(qe[2,0] + qe[3,0]) - L2_x*L3_y*m4*np.sin(qe[2,0]) + L2_y*l3_y*m3*np.cos(qe[2,0]) - L2_x*l3_y*m3*np.sin(qe[2,0])) - l4_y*m4*dqe[3,0]*(L2_x*np.sin(qe[2,0] + qe[3,0]) - L2_y*np.cos(qe[2,0] + qe[3,0]) + L3_y*np.sin(qe[3,0])))

    P23e = (L2_y*l4_y*m4*dqe[1,0]*np.cos(qe[2,0] + qe[3,0]) + L2_y*l4_y*m4*dqe[2,0]*np.cos(qe[2,0] + qe[3,0]) + L2_y*l4_y*m4*dqe[3,0]*np.cos(qe[2,0] + qe[3,0]) + L2_y*L3_y*m4*dqe[1,0]*np.cos(qe[2,0]) + L2_y*L3_y*m4*dqe[2,0]*np.cos(qe[2,0]) - L2_x*l4_y*m4*dqe[1,0]*np.sin(qe[2,0] + qe[3,0]) - L2_x*l4_y*m4*dqe[2,0]*np.sin(qe[2,0] + qe[3,0]) - L2_x*l4_y*m4*dqe[3,0]*np.sin(qe[2,0] + qe[3,0]) - L2_x*L3_y*m4*dqe[1,0]*np.sin(qe[2,0]) - L2_x*L3_y*m4*dqe[2,0]*np.sin(qe[2,0]) + L2_y*l3_y*m3*dqe[1,0]*np.cos(qe[2,0]) + L2_y*l3_y*m3*dqe[2,0]*np.cos(qe[2,0]) - L2_x*l3_y*m3*dqe[1,0]*np.sin(qe[2,0]) - L2_x*l3_y*m3*dqe[2,0]*np.sin(qe[2,0]) - L3_y*l4_y*m4*dqe[3,0]*np.sin(qe[3,0]))

    P24e = (-l4_y*m4*(L2_x*np.sin(qe[2,0] + qe[3,0]) - L2_y*np.cos(qe[2,0] + qe[3,0]) + L3_y*np.sin(qe[3,0]))*(dqe[1,0] + dqe[2,0] + dqe[3,0]))

    P31e = -P13e

    P32e = (L2_x*l4_y*m4*dqe[1,0]*np.sin(qe[2,0] + qe[3,0]) - L2_y*L3_y*m4*dqe[1,0]*np.cos(qe[2,0]) - L2_y*l4_y*m4*dqe[1,0]*np.cos(qe[2,0] + qe[3,0]) + L2_x*L3_y*m4*dqe[1,0]*np.sin(qe[2,0]) - L2_y*l3_y*m3*dqe[1,0]*np.cos(qe[2,0]) + L2_x*l3_y*m3*dqe[1,0]*np.sin(qe[2,0]) - L3_y*l4_y*m4*dqe[3,0]*np.sin(qe[3,0]))

    P33e = (Keq + -L3_y*l4_y*m4*dqe[3,0]*np.sin(qe[3,0]))

    P34e = (-L3_y*l4_y*m4*np.sin(qe[3,0])*(dqe[1,0] + dqe[2,0] + dqe[3,0]))

    P41e = -P14e

    P42e = (l4_y*m4*(L3_y*dqe[1,0]*np.sin(qe[3,0]) + L3_y*dqe[2,0]*np.sin(qe[3,0]) - L2_y*dqe[1,0]*np.cos(qe[2,0] + qe[3,0]) + L2_x*dqe[1,0]*np.sin(qe[2,0] + qe[3,0])))

    P43e = (L3_y*l4_y*m4*np.sin(qe[3,0])*(dqe[1,0] + dqe[2,0]))

    P44e = (Keq)

    Pe  = U_lin*np.array([[P11e, P12e, P13e, P14e], 
                         [P21e, P22e, P23e, P24e], 
                         [P31e, P32e, P33e, P34e],
                         [P41e, P42e, P43e, P44e]])

    ##########

    D2e = (g*l2_y*m2*np.sin(qe[1,0]) - g*m4*(L3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0]) + l4_y*np.cos(qe[1,0] + qe[2,0] + qe[3,0])) - g*m3*(l3_y*np.cos(qe[1,0] + qe[2,0]) + L2_x*np.cos(qe[1,0]) - L2_y*np.sin(qe[1,0])))
    D3e = (- g*m4*(L3_y*np.cos(qe[1,0] + qe[2,0]) + l4_y*np.cos(qe[1,0] + qe[2,0] + qe[3,0])) - g*l3_y*m3*np.cos(qe[1,0] + qe[2,0]))
    D4e = (-g*l4_y*m4*np.cos(qe[1,0] + qe[2,0] + qe[3,0]))

    Ge = U_lin*np.array([[0], [D2e], [D3e], [D4e]])
    
    # LEY DE CONTROL 
    u =(Me@ddqd + Pe@dqr + Ge - Kd@(dqe - dqr) - K1@z1)
    
    # Limitador de corriente:
    if u[0] > u_lim or u[0] < -u_lim:
        u[0] = u_pas[0]
    else:
        u_pas[0] = u[0]

    if u[1] > u_lim or u[1] < -u_lim:
        u[1] = u_pas[1]
    else:
        u_pas[1] = u[1]
    #Almacenar variables de control:

    U1[0,k] = u[0,0]
    U2[0,k] = u[1,0]
    U3[0,k] = u[2,0]
    U4[0,k] = u[3,0]


    # MODELO DINAMICO DEL SISTEMA 
    M11 = (Jeq+Ixx3 + Ixx4 + Izz1 + Izz2 + m3*(l3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]))**2 - Ixx4*np.sin(q[1,0] + q[2,0] + q[3,0])**2 + Izz4*np.sin(q[1,0] + q[2,0] + q[3,0])**2 - Ixx3*np.sin(q[1,0] + q[2,0])**2 + Izz3*np.sin(q[1,0] + q[2,0])**2 + m4*(L3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]) + l4_y*np.cos(q[1,0] + q[2,0] + q[3,0]))**2 + Ixx2*np.sin(q[1,0])**2 - Izz2*np.sin(q[1,0])**2 + l2_y**2*m2*np.sin(q[1,0])**2)

    M22 = (Jeq+Iyy2 + Iyy3 + Iyy4 + L2_x**2*m3 + L2_x**2*m4 + L2_y**2*m3 + L2_y**2*m4 + L3_y**2*m4 + l2_y**2*m2 + l3_y**2*m3 + l4_y**2*m4 + 2*L2_x*l4_y*m4*np.cos(q[2,0] + q[3,0]) + 2*L2_x*L3_y*m4*np.cos(q[2,0]) + 2*L2_y*l4_y*m4*np.sin(q[2,0] + q[3,0]) + 2*L2_y*L3_y*m4*np.sin(q[2,0]) + 2*L2_x*l3_y*m3*np.cos(q[2,0]) + 2*L3_y*l4_y*m4*np.cos(q[3,0]) + 2*L2_y*l3_y*m3*np.sin(q[2,0]))

    M23= (Iyy3 + Iyy4 + L3_y**2*m4 + l3_y**2*m3 + l4_y**2*m4 + L2_x*l4_y*m4*np.cos(q[2,0] + q[3,0]) + L2_x*L3_y*m4*np.cos(q[2,0]) + L2_y*l4_y*m4*np.sin(q[2,0] + q[3,0]) + L2_y*L3_y*m4*np.sin(q[2,0]) + L2_x*l3_y*m3*np.cos(q[2,0]) + 2*L3_y*l4_y*m4*np.cos(q[3,0]) + L2_y*l3_y*m3*np.sin(q[2,0]))

    M24 = (Iyy4 + l4_y*m4*(l4_y + L2_x*np.cos(q[2,0] + q[3,0]) + L2_y*np.sin(q[2,0] + q[3,0]) + L3_y*np.cos(q[3,0])))

    M32 = (Iyy3 + Iyy4 + L3_y**2*m4 + l3_y**2*m3 + l4_y**2*m4 + L2_x*l4_y*m4*np.cos(q[2,0] + q[3,0]) + L2_x*L3_y*m4*np.cos(q[2,0]) + L2_y*l4_y*m4*np.sin(q[2,0] + q[3,0]) + L2_y*L3_y*m4*np.sin(q[2,0]) + L2_x*l3_y*m3*np.cos(q[2,0]) + 2*L3_y*l4_y*m4*np.cos(q[3,0]) + L2_y*l3_y*m3*np.sin(q[2,0]))

    M33 =  (Jeq+Iyy3 + Iyy4 + m4*(L3_y**2 + 2*np.cos(q[3,0])*L3_y*l4_y + l4_y**2) + l3_y**2*m3)

    M34 = (Iyy4 + m4*(l4_y**2 + L3_y*np.cos(q[3,0])*l4_y))

    M42= (Iyy4 + l4_y*m4*(l4_y + L2_x*np.cos(q[2,0] + q[3,0]) + L2_y*np.sin(q[2,0] + q[3,0]) + L3_y*np.cos(q[3,0])))

    M43=  (Iyy4 + m4*(l4_y**2 + L3_y*np.cos(q[3,0])*l4_y))

    M44=  (Jeq+ m4*l4_y**2 + Iyy4)

    M = U_lin*np.array([[M11  ,  0   ,     0   ,    0] ,
                  [0 ,   M22  ,   M23   ,M24],
                  [0   , M32  ,   M33 ,  M34],
                  [0  ,  M42   ,  M43 ,  M44]])
    ############
    P11 = (Keq - dq[2,0]*((Ixx3*np.sin(2*q[1,0] + 2*q[2,0]))/2 - (Izz3*np.sin(2*q[1,0] + 2*q[2,0]))/2 + (Ixx4*np.sin(2*q[1,0] + 2*q[2,0] + 2*q[3,0]))/2 - (Izz4*np.sin(2*q[1,0] + 2*q[2,0] + 2*q[3,0]))/2 + m4*(L3_y*np.sin(q[1,0] + q[2,0]) + l4_y*np.sin(q[1,0] + q[2,0] + q[3,0]))*(L3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]) + l4_y*np.cos(q[1,0] + q[2,0] + q[3,0])) + l3_y*m3*np.sin(q[1,0] + q[2,0])*(l3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]))) - dq[1,0]*((Ixx3*np.sin(2*q[1,0] + 2*q[2,0]))/2 - (Izz3*np.sin(2*q[1,0] + 2*q[2,0]))/2 - (Ixx2*np.sin(2*q[1,0]))/2 + (Izz2*np.sin(2*q[1,0]))/2 + (Ixx4*np.sin(2*q[1,0] + 2*q[2,0] + 2*q[3,0]))/2 - (Izz4*np.sin(2*q[1,0] + 2*q[2,0] + 2*q[3,0]))/2 + m4*(L3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]) + l4_y*np.cos(q[1,0] + q[2,0] + q[3,0]))*(L3_y*np.sin(q[1,0] + q[2,0]) + L2_y*np.cos(q[1,0]) + L2_x*np.sin(q[1,0]) + l4_y*np.sin(q[1,0] + q[2,0] + q[3,0])) + m3*(l3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]))*(L2_y*np.cos(q[1,0]) + l3_y*np.sin(q[1,0] + q[2,0]) + L2_x*np.sin(q[1,0])) - (l2_y**2*m2*np.sin(2*q[1,0]))/2) - dq[3,0]*np.sin(q[1,0] + q[2,0] + q[3,0])*(Ixx4*np.cos(q[1,0] + q[2,0] + q[3,0]) - Izz4*np.cos(q[1,0] + q[2,0] + q[3,0]) + l4_y**2*m4*np.cos(q[1,0] + q[2,0] + q[3,0]) + L3_y*l4_y*m4*np.cos(q[1,0] + q[2,0]) + L2_x*l4_y*m4*np.cos(q[1,0]) - L2_y*l4_y*m4*np.sin(q[1,0])))

    P12 = (-dq[0,0]*((Ixx3*np.sin(2*q[1,0] + 2*q[2,0]))/2 - (Izz3*np.sin(2*q[1,0] + 2*q[2,0]))/2 - (Ixx2*np.sin(2*q[1,0]))/2 + (Izz2*np.sin(2*q[1,0]))/2 + (Ixx4*np.sin(2*q[1,0] + 2*q[2,0] + 2*q[3,0]))/2 - (Izz4*np.sin(2*q[1,0] + 2*q[2,0] + 2*q[3,0]))/2 + m4*(L3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]) + l4_y*np.cos(q[1,0] + q[2,0] + q[3,0]))*(L3_y*np.sin(q[1,0] + q[2,0]) + L2_y*np.cos(q[1,0]) + L2_x*np.sin(q[1,0]) + l4_y*np.sin(q[1,0] + q[2,0] + q[3,0])) + m3*(l3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]))*(L2_y*np.cos(q[1,0]) + l3_y*np.sin(q[1,0] + q[2,0]) + L2_x*np.sin(q[1,0])) - (l2_y**2*m2*np.sin(2*q[1,0]))/2))

    P13 = (-dq[0,0]*((Ixx3*np.sin(2*q[1,0] + 2*q[2,0]))/2 - (Izz3*np.sin(2*q[1,0] + 2*q[2,0]))/2 + (Ixx4*np.sin(2*q[1,0] + 2*q[2,0] + 2*q[3,0]))/2 - (Izz4*np.sin(2*q[1,0] + 2*q[2,0] + 2*q[3,0]))/2 + m4*(L3_y*np.sin(q[1,0] + q[2,0]) + l4_y*np.sin(q[1,0] + q[2,0] + q[3,0]))*(L3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]) + l4_y*np.cos(q[1,0] + q[2,0] + q[3,0])) + l3_y*m3*np.sin(q[1,0] + q[2,0])*(l3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]))))

    P14 = (-dq[0,0]*np.sin(q[1,0] + q[2,0] + q[3,0])*(Ixx4*np.cos(q[1,0] + q[2,0] + q[3,0]) - Izz4*np.cos(q[1,0] + q[2,0] + q[3,0]) + l4_y**2*m4*np.cos(q[1,0] + q[2,0] + q[3,0]) + L3_y*l4_y*m4*np.cos(q[1,0] + q[2,0]) + L2_x*l4_y*m4*np.cos(q[1,0]) - L2_y*l4_y*m4*np.sin(q[1,0])))

    P21 = -P12

    P22 = (Keq + dq[2,0]*(L2_y*l4_y*m4*np.cos(q[2,0] + q[3,0]) + L2_y*L3_y*m4*np.cos(q[2,0]) - L2_x*l4_y*m4*np.sin(q[2,0] + q[3,0]) - L2_x*L3_y*m4*np.sin(q[2,0]) + L2_y*l3_y*m3*np.cos(q[2,0]) - L2_x*l3_y*m3*np.sin(q[2,0])) - l4_y*m4*dq[3,0]*(L2_x*np.sin(q[2,0] + q[3,0]) - L2_y*np.cos(q[2,0] + q[3,0]) + L3_y*np.sin(q[3,0])))

    P23 = (L2_y*l4_y*m4*dq[1,0]*np.cos(q[2,0] + q[3,0]) + L2_y*l4_y*m4*dq[2,0]*np.cos(q[2,0] + q[3,0]) + L2_y*l4_y*m4*dq[3,0]*np.cos(q[2,0] + q[3,0]) + L2_y*L3_y*m4*dq[1,0]*np.cos(q[2,0]) + L2_y*L3_y*m4*dq[2,0]*np.cos(q[2,0]) - L2_x*l4_y*m4*dq[1,0]*np.sin(q[2,0] + q[3,0]) - L2_x*l4_y*m4*dq[2,0]*np.sin(q[2,0] + q[3,0]) - L2_x*l4_y*m4*dq[3,0]*np.sin(q[2,0] + q[3,0]) - L2_x*L3_y*m4*dq[1,0]*np.sin(q[2,0]) - L2_x*L3_y*m4*dq[2,0]*np.sin(q[2,0]) + L2_y*l3_y*m3*dq[1,0]*np.cos(q[2,0]) + L2_y*l3_y*m3*dq[2,0]*np.cos(q[2,0]) - L2_x*l3_y*m3*dq[1,0]*np.sin(q[2,0]) - L2_x*l3_y*m3*dq[2,0]*np.sin(q[2,0]) - L3_y*l4_y*m4*dq[3,0]*np.sin(q[3,0]))

    P24 = (-l4_y*m4*(L2_x*np.sin(q[2,0] + q[3,0]) - L2_y*np.cos(q[2,0] + q[3,0]) + L3_y*np.sin(q[3,0]))*(dq[1,0] + dq[2,0] + dq[3,0]))

    P31 = -P13

    P32 = (L2_x*l4_y*m4*dq[1,0]*np.sin(q[2,0] + q[3,0]) - L2_y*L3_y*m4*dq[1,0]*np.cos(q[2,0]) - L2_y*l4_y*m4*dq[1,0]*np.cos(q[2,0] + q[3,0]) + L2_x*L3_y*m4*dq[1,0]*np.sin(q[2,0]) - L2_y*l3_y*m3*dq[1,0]*np.cos(q[2,0]) + L2_x*l3_y*m3*dq[1,0]*np.sin(q[2,0]) - L3_y*l4_y*m4*dq[3,0]*np.sin(q[3,0]))

    P33 = (Keq -L3_y*l4_y*m4*dq[3,0]*np.sin(q[3,0]))

    P34 = (-L3_y*l4_y*m4*np.sin(q[3,0])*(dq[1,0] + dq[2,0] + dq[3,0]))

    P41 = -P14

    P42 = (l4_y*m4*(L3_y*dq[1,0]*np.sin(q[3,0]) + L3_y*dq[2,0]*np.sin(q[3,0]) - L2_y*dq[1,0]*np.cos(q[2,0] + q[3,0]) + L2_x*dq[1,0]*np.sin(q[2,0] + q[3,0])))

    P43 = (L3_y*l4_y*m4*np.sin(q[3,0])*(dq[1,0] + dq[2,0]))

    P44 = (Keq)

    P  = U_lin*np.array([[P11, P12, P13, P14], 
                [P21, P22, P23, P24], 
                [P31, P32, P33, P34],
                [P41, P42, P43, P44]])

    ##########

    D2 = (g*l2_y*m2*np.sin(q[1,0]) - g*m4*(L3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0]) + l4_y*np.cos(q[1,0] + q[2,0] + q[3,0])) - g*m3*(l3_y*np.cos(q[1,0] + q[2,0]) + L2_x*np.cos(q[1,0]) - L2_y*np.sin(q[1,0])))
    D3 = (- g*m4*(L3_y*np.cos(q[1,0] + q[2,0]) + l4_y*np.cos(q[1,0] + q[2,0] + q[3,0])) - g*l3_y*m3*np.cos(q[1,0] + q[2,0]))
    D4 = (-g*l4_y*m4*np.cos(q[1,0] + q[2,0] + q[3,0]))

    d = U_lin*np.array([[0], [D2], [D3], [D4]])

    dq = dq + T * np.linalg.solve(M, u - np.dot(P, dq) - d)
    q = q + T * dq
    Q1[0,k]=q[0,0]
    Q2[0,k]=q[1,0]
    Q3[0,k]=q[2,0]
    Q4[0,k]=q[3,0]
    
    end_time = time.time()
    execution_time = end_time - start_time
    if execution_time > execution_time_max:
        execution_time_max = execution_time


print("Tiempo de ejecución maximo: ", execution_time_max, " segundos")

# GRAFICOS
# Graficar Q1 y Qd1
Qd1p=np.transpose(Qd1)
Q1p=np.transpose(Q1)
U1p=np.transpose(U1)
Qd2p=np.transpose(Qd2)
Q2p=np.transpose(Q2)
U2p=np.transpose(U2)
Qd3p=np.transpose(Qd3)
Q3p=np.transpose(Q3)
U3p=np.transpose(U3)
Qd4p=np.transpose(Qd4)
Q4p=np.transpose(Q4)
U4p=np.transpose(U4)
ejex = np.linspace(0, nn*T, nn)
print("Plots:\n")

plt.subplot(811)
plt.plot(ejex, Qd1p, ejex, Q1p)
plt.grid(True)
plt.ylabel('q1 [rad]')

# Graficar U1
plt.subplot(812)
plt.plot(ejex, U1p ,'m')
plt.grid(True)
plt.xlabel('TIEMPO   [s]')
plt.ylabel('u1 [V]')

# Graficar Q2 y Qd2
plt.subplot(813)
plt.plot(ejex, Qd2p, ejex, Q2p)
plt.grid(True)
plt.xlabel('TIEMPO   [s]')
plt.ylabel('q2 [rad]')

# Graficar U2
plt.subplot(814)
plt.plot(ejex, U2p, 'm')
plt.grid(True)
plt.xlabel('TIEMPO   [s]')
plt.ylabel('u2 [V]')

# Graficar Q3 y Qd3
plt.subplot(815)
plt.plot(ejex, Qd3p, ejex, Q3p)
plt.grid(True)
plt.xlabel('TIEMPO   [s]')
plt.ylabel('q3 [rad]')

# Graficar U3
plt.subplot(816)
plt.plot(ejex, U3p, 'm')
plt.grid(True)
plt.xlabel('TIEMPO   [s]')
plt.ylabel('u3 [V]')

# Graficar Q4 y Qd4
plt.subplot(817)
plt.plot(ejex, Qd4p, ejex, Q4p)
plt.grid(True)
plt.xlabel('TIEMPO   [s]')
plt.ylabel('q4 [rad]')

# Graficar U4
plt.subplot(818)
plt.plot(ejex, U4p, 'm')
plt.grid(True)
plt.xlabel('TIEMPO   [s]')
plt.ylabel('u4 [V]')
plt.show()