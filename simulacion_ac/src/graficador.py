#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

# Variables globales para almacenar los datos recibidos
# global x_tiempo, y_qdes1, refresh_point, qdes_vec
x_tiempo = []
y_qdes1 = []
y_qdes2 = []
qdes_vec = [0,0]
y_qreal1 = []
y_qreal2 = []
qreal_vec = [0,0]
y_u1 = []
y_u2 = []
u_vec = [0,0]
refresh_point = 500    # Punto en el cual se borra la data pasada y se actualiza
def callback_q_des(q_des):
    global qdes_vec
    qdes_vec=q_des.data

def callback_u_sig(u_sig):
    global u_vec
    u_vec =u_sig.data

def callback_q_real(q_real):
    global qreal_vec
    qreal_vec=q_real.data
    
def create_point():
    global x_tiempo, y_qdes1, y_qdes2, refresh_point, qdes_vec
    global y_qreal1, y_qreal2, qreal_vec, y_u1, y_u2, u_vec
    current_time = rospy.Time.now().to_sec()  # Obtener el tiempo actual en segundos
    q_des1=qdes_vec[0]
    q_des2=qdes_vec[1]
    qreal1=qreal_vec[0]
    qreal2=qreal_vec[1]
    u1=u_vec[0]
    u2=u_vec[1]
    x_tiempo.append(current_time)  # Tiempo actual como eje x
    y_qdes1.append(q_des1)
    y_qdes2.append(q_des2)
    y_qreal1.append(qreal1)
    y_qreal2.append(qreal2)
    y_u1.append(u1)
    y_u2.append(u2)

    # Actualizar y borrar la data pasada cuando se alcanza el refresh_point
    if len(x_tiempo) >= refresh_point:
        x_tiempo = x_tiempo[-refresh_point:]
        y_qdes1  = y_qdes1[-refresh_point:]
        y_qdes2  = y_qdes2[-refresh_point:]
        y_qreal1 = y_qreal1[-refresh_point:]
        y_qreal2 = y_qreal2[-refresh_point:]
        y_u1     = y_u1[-refresh_point:]
        y_u2     = y_u2[-refresh_point:]

    
if __name__ == "__main__":

    rospy.init_node('q_graph')
    rospy.Subscriber('q_des',Float64MultiArray,callback_q_des)
    rospy.Subscriber('pos_present_value',Float64MultiArray,callback_q_real)
    rospy.Subscriber('u',Float64MultiArray,callback_u_sig)

    plt.ion()  # Modo interactivo de matplotlib
    fig, (ax1,ax2,ax3,ax4) = plt.subplots(4,1)  # Crear una figura y un conjunto de ejes
    line1, = ax1.plot(x_tiempo, y_qdes1)  # Crear una línea para graficar 1 los datos
    line2, = ax1.plot(x_tiempo, y_qreal1)
    line3, = ax2.plot(x_tiempo, y_u1, 'm')
    line4, = ax3.plot(x_tiempo, y_qdes2)  # Crear una línea para graficar 2 los datos
    line5, = ax3.plot(x_tiempo, y_qreal2)
    line6, = ax4.plot(x_tiempo, y_u2, 'm') 

    ax1.set_ylabel('q1 [rad]')
    ax1.set_title('Respuest temporal')

    ax2.set_ylabel('u1 [rad]')

    ax3.set_ylabel('q2 [rad]')

    ax4.set_xlabel('Tiempo')
    ax4.set_ylabel('q1 [rad]')


    while not rospy.is_shutdown():
        create_point()
        if len(x_tiempo) == len(y_qdes1):
            line1.set_xdata(x_tiempo)  # Actualizar los datos del eje x
            line1.set_ydata(y_qdes1)  # Actualizar los datos del eje y
            line2.set_xdata(x_tiempo)  # Actualizar los datos del eje x
            line2.set_ydata(y_qreal1)

            line3.set_xdata(x_tiempo)  # Actualizar los datos del eje x
            line3.set_ydata(y_u1)  # Actualizar los datos del eje y

            line4.set_xdata(x_tiempo) 
            line4.set_ydata(y_qdes2)
            line5.set_xdata(x_tiempo)
            line5.set_ydata(y_qreal2)

            line6.set_xdata(x_tiempo) 
            line6.set_ydata(y_u2)

            ax1.relim()  # Actualizar los límites de los ejes
            ax1.autoscale_view()  # Ajustar la escala de los ejes automáticamente

            ax2.relim()  # Actualizar los límites de los ejes
            ax2.autoscale_view()  # Ajustar la escala de los ejes automáticamente

            ax3.relim()  # Actualizar los límites de los ejes
            ax3.autoscale_view()  # Ajustar la escala de los ejes automáticamente

            ax4.relim()  # Actualizar los límites de los ejes
            ax4.autoscale_view()  # Ajustar la escala de los ejes automáticamente

            plt.pause(0.01)  # Pausar para permitir la actualización del gráfico


    plt.ioff()  # Desactivar el modo interactivo de matplotlib
    plt.show()


