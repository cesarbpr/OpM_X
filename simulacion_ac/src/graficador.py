#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

# Variables globales para almacenar los datos recibidos
x_data = []
y_data = []
q_temp = ([0,0])
def cb_function(msg):
    global x_data, y_data, q_temp
    current_time = rospy.Time.now().to_sec()  # Obtener el tiempo actual en segundos
    x_data.append(current_time)  # Tiempo actual como eje x
    q_temp = msg.data
    print(q_temp[1])
    y_data.append(q_temp[1]) 
    #rospy.loginfo(msg)

if __name__ == "__main__":

    rospy.init_node('q_graph')
    rospy.Subscriber('q_ref',Float64MultiArray,cb_function)

    plt.ion()  # Modo interactivo de matplotlib
    fig, ax = plt.subplots()  # Crear una figura y un conjunto de ejes
    line, = ax.plot(x_data, y_data)  # Crear una línea para graficar los datos



    plt.xlabel('Tiempo')
    plt.ylabel('Valor')
    plt.title('Grafica en tiempo real')

    while not rospy.is_shutdown():
        line.set_xdata(x_data)  # Actualizar los datos del eje x
        line.set_ydata(y_data)  # Actualizar los datos del eje y
        ax.relim()  # Actualizar los límites de los ejes
        ax.autoscale_view()  # Ajustar la escala de los ejes automáticamente
        plt.pause(0.01)  # Pausar para permitir la actualización del gráfico


    plt.ioff()  # Desactivar el modo interactivo de matplotlib
    plt.show()


