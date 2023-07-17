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
        self.qreal_vec = [0, 0, 0, 0, 0]
        self.y_u1 = []
        self.y_u2 = []
        self.u_vec = [0, 0, 0, 0, 0]
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
        qreal1 = self.qreal_vec[2]
        qreal2 = self.qreal_vec[3]
        u1 = self.u_vec[2]
        u2 = self.u_vec[3]
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

