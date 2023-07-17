#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

class Graficar:
    def __init__(self):
        self.x_tiempo = []
        self.y_qdes1 = []
        self.y_qdes2 = []
        self.qdes_vec = [0, 0]
        self.y_qreal1 = []
        self.y_qreal2 = []
        self.qreal_vec = [0, 0]
        self.y_u1 = []
        self.y_u2 = []
        self.u_vec = [0, 0]
        self.refresh_point = 500

        rospy.init_node('q_graph')
        rospy.Subscriber('q_des', Float64MultiArray, self.callback_q_des)
        rospy.Subscriber('pos_present_value', Float64MultiArray, self.callback_q_real)
        rospy.Subscriber('u', Float64MultiArray, self.callback_u_sig)

        plt.ion()
        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(4, 1)
        self.line1, = self.ax1.plot(self.x_tiempo, self.y_qdes1)
        self.line2, = self.ax1.plot(self.x_tiempo, self.y_qreal1)
        self.line3, = self.ax2.plot(self.x_tiempo, self.y_u1, 'm')
        self.line4, = self.ax3.plot(self.x_tiempo, self.y_qdes2)
        self.line5, = self.ax3.plot(self.x_tiempo, self.y_qreal2)
        self.line6, = self.ax4.plot(self.x_tiempo, self.y_u2, 'm')

        self.ax1.set_ylabel('q1 [rad]')
        self.ax1.set_title('Respuesta temporal')
        self.ax2.set_ylabel('i1 [mA]')
        self.ax3.set_ylabel('q2 [rad]')
        self.ax4.set_xlabel('Tiempo')
        self.ax4.set_ylabel('i2 [mA]')

        self.ax1.grid(True)
        self.ax2.grid(True)
        self.ax3.grid(True)
        self.ax4.grid(True)

    def callback_q_des(self, q_des):
        self.qdes_vec = q_des.data

    def callback_u_sig(self, u_sig):
        self.u_vec = u_sig.data

    def callback_q_real(self, q_real):
        self.qreal_vec = q_real.data

    def create_point(self):
        current_time = rospy.Time.now().to_sec()
        q_des1 = self.qdes_vec[0]
        q_des2 = self.qdes_vec[1]
        qreal1 = self.qreal_vec[0]
        qreal2 = self.qreal_vec[1]
        u1 = self.u_vec[0]
        u2 = self.u_vec[1]
        self.x_tiempo.append(current_time)
        self.y_qdes1.append(q_des1)
        self.y_qdes2.append(q_des2)
        self.y_qreal1.append(qreal1)
        self.y_qreal2.append(qreal2)
        self.y_u1.append(u1)
        self.y_u2.append(u2)

        if len(self.x_tiempo) >= self.refresh_point:
            self.x_tiempo = self.x_tiempo[-self.refresh_point:]
            self.y_qdes1 = self.y_qdes1[-self.refresh_point:]
            self.y_qdes2 = self.y_qdes2[-self.refresh_point:]
            self.y_qreal1 = self.y_qreal1[-self.refresh_point:]
            self.y_qreal2 = self.y_qreal2[-self.refresh_point:]
            self.y_u1 = self.y_u1[-self.refresh_point:]
            self.y_u2 = self.y_u2[-self.refresh_point:]

    def plot_graph(self):
        self.create_point()
        if len(self.x_tiempo) == len(self.y_qdes1):
            self.line1.set_xdata(self.x_tiempo)
            self.line1.set_ydata(self.y_qdes1)
            self.line2.set_xdata(self.x_tiempo)
            self.line2.set_ydata(self.y_qreal1)
            self.line3.set_xdata(self.x_tiempo)
            self.line3.set_ydata(self.y_u1)
            self.line4.set_xdata(self.x_tiempo)
            self.line4.set_ydata(self.y_qdes2)
            self.line5.set_xdata(self.x_tiempo)
            self.line5.set_ydata(self.y_qreal2)
            self.line6.set_xdata(self.x_tiempo)
            self.line6.set_ydata(self.y_u2)

            self.ax1.relim()
            self.ax1.autoscale_view()
            self.ax2.relim()
            self.ax2.autoscale_view()
            self.ax3.relim()
            self.ax3.autoscale_view()
            self.ax4.relim()
            self.ax4.autoscale_view()

            plt.pause(0.01)

    def run(self):
        while not rospy.is_shutdown():
            self.plot_graph()

        plt.ioff()
        plt.show()


if __name__ == "__main__":
    graficar = Graficar()
    graficar.run()




'''
# Variables globales para almacenar los datos recibidos
# global x_tiempo, y_qdes1, refresh_point, qdes_vec
x_tiempo = []
y_qdes1 = []
y_qdes2 = []
qdes_vec = [0,0]
y_qreal1 = []
y_qreal2 = []
qreal_vec = [0,0]
#qreal_vec = [0,0,0,0,0]
y_u1 = []
y_u2 = []
#u_vec = [0,0,0,0,0]
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

    ax2.set_ylabel('i1 [mA]')

    ax3.set_ylabel('q2 [rad]')

    ax4.set_xlabel('Tiempo')
    ax4.set_ylabel('i2 [mA]')

    ax1.grid(True)  # Agregar grid al primer eje
    ax2.grid(True)  # Agregar grid al segundo eje
    ax3.grid(True)  # Agregar grid al tercer eje
    ax4.grid(True)  # Agregar grid al cuarto eje

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
'''

