#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

# Variables globales para almacenar los datos recibidos
global x_data, y_data, refresh_point, q_vec
x_data = []
y_data = []
q_vec = [0,0]
refresh_point = 500    # Punto en el cual se borra la data pasada y se actualiza
def cb_function(q_des):
    global q_vec
    q_vec=q_des.data
    #rospy.loginfo(msg)

def create_point():
    global x_data, y_data, refresh_point, q_vec
    current_time = rospy.Time.now().to_sec()  # Obtener el tiempo actual en segundos
    q_des1=q_vec[0]
    q_des2=q_vec[1]
    x_data.append(current_time)  # Tiempo actual como eje x
    y_data.append(q_des1)
        # Actualizar y borrar la data pasada cuando se alcanza el refresh_point
    if len(x_data) >= refresh_point:
        x_data = x_data[-refresh_point:]
        y_data = y_data[-refresh_point:]
if __name__ == "__main__":

    rospy.init_node('q_graph')
    rospy.Subscriber('q_des',Float64MultiArray,cb_function)

    plt.ion()  # Modo interactivo de matplotlib
    fig, ax = plt.subplots()  # Crear una figura y un conjunto de ejes
    line, = ax.plot(x_data, y_data)  # Crear una línea para graficar los datos

    plt.xlabel('Tiempo')
    plt.ylabel('Valor')
    plt.title('Grafica en tiempo real')

    while not rospy.is_shutdown():
        create_point()
        if len(x_data) == len(y_data):
            line.set_xdata(x_data)  # Actualizar los datos del eje x
            line.set_ydata(y_data)  # Actualizar los datos del eje y
            ax.relim()  # Actualizar los límites de los ejes
            ax.autoscale_view()  # Ajustar la escala de los ejes automáticamente
            plt.pause(0.01)  # Pausar para permitir la actualización del gráfico


    plt.ioff()  # Desactivar el modo interactivo de matplotlib
    plt.show()


