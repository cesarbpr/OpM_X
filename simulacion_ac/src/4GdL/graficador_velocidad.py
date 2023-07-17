#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray


class Graficar:



    def __init__(self):

        self.x_tiempo = []
        self.y_qdes1 = []
        self.y_qdes2 = []
        self.y_qdes3 = []
        self.y_qdes4 = []
        self.qdes_vec = [0, 0, 0, 0]
        self.y_qreal1 = []
        self.y_qreal2 = []
        self.y_qreal3 = []
        self.y_qreal4 = []
        self.qreal_vec = [0, 0, 0, 0, 0]
        self.y_u1 = []
        self.y_u2 = []
        self.y_u3 = []
        self.y_u4 = []
        self.u_vec = [0, 0, 0, 0]
        self.refresh_point = 100
        rospy.init_node('q_graph')
        rospy.Subscriber('dqe', Float64MultiArray, self.callback_q_des)
        rospy.Subscriber('vel_present_value', Float64MultiArray, self.callback_q_real)

        plt.ion()
        self.fig, (self.ax1, self.ax3, self.ax5, self.ax7) = plt.subplots(4, 1)
        self.line1, = self.ax1.plot(self.x_tiempo, self.y_qdes1)
        self.line2, = self.ax1.plot(self.x_tiempo, self.y_qreal1)
        self.line4, = self.ax3.plot(self.x_tiempo, self.y_qdes2)
        self.line5, = self.ax3.plot(self.x_tiempo, self.y_qreal2)
        self.line7, = self.ax5.plot(self.x_tiempo, self.y_qdes3)
        self.line8, = self.ax5.plot(self.x_tiempo, self.y_qreal3)
        self.line10, = self.ax7.plot(self.x_tiempo, self.y_qdes4)
        self.line11, = self.ax7.plot(self.x_tiempo, self.y_qreal4)

        self.ax1.set_ylabel('dq1 [rad/s]')
        # self.ax1.set_title('Respuesta temporal')
        self.ax3.set_ylabel('dq2 [rad/s]]')
        self.ax5.set_ylabel('dq3 [rad/s]]')
        self.ax7.set_ylabel('dq4 [rad/s]]')
        self.ax7.set_xlabel('Tiempo [s]')

        self.ax1.grid(True)
        self.ax3.grid(True)
        self.ax5.grid(True)
        self.ax7.grid(True)

    def callback_q_des(self, q_des):
        self.qdes_vec = q_des.data

    def callback_q_real(self, q_real):
        self.qreal_vec = q_real.data

    def create_point(self):
        current_time = rospy.Time.now().to_sec()
        q_des1 = self.qdes_vec[0]
        q_des2 = self.qdes_vec[1]
        q_des3 = self.qdes_vec[2]
        q_des4 = self.qdes_vec[3]
        qreal1 = self.qreal_vec[0]
        qreal2 = self.qreal_vec[1]
        qreal3 = self.qreal_vec[2]
        qreal4 = self.qreal_vec[3]

        self.x_tiempo.append(current_time)
        self.y_qdes1.append(q_des1)
        self.y_qdes2.append(q_des2)
        self.y_qdes3.append(q_des3)
        self.y_qdes4.append(q_des4)
        self.y_qreal1.append(qreal1)
        self.y_qreal2.append(qreal2)
        self.y_qreal3.append(qreal3)
        self.y_qreal4.append(qreal4)


        if len(self.x_tiempo) >= self.refresh_point:
            self.x_tiempo = self.x_tiempo[-self.refresh_point:]
            self.y_qdes1 = self.y_qdes1[-self.refresh_point:]
            self.y_qdes2 = self.y_qdes2[-self.refresh_point:]
            self.y_qdes3 = self.y_qdes3[-self.refresh_point:]
            self.y_qdes4 = self.y_qdes4[-self.refresh_point:]
            self.y_qreal1 = self.y_qreal1[-self.refresh_point:]
            self.y_qreal2 = self.y_qreal2[-self.refresh_point:]
            self.y_qreal3 = self.y_qreal3[-self.refresh_point:]
            self.y_qreal4 = self.y_qreal4[-self.refresh_point:]


    def plot_graph(self):
        self.create_point()
        if len(self.x_tiempo) == len(self.y_qdes1):
            self.line1.set_xdata(self.x_tiempo)
            self.line1.set_ydata(self.y_qdes1)
            self.line2.set_xdata(self.x_tiempo)
            self.line2.set_ydata(self.y_qreal1)
            self.line4.set_xdata(self.x_tiempo)
            self.line4.set_ydata(self.y_qdes2)
            self.line5.set_xdata(self.x_tiempo)
            self.line5.set_ydata(self.y_qreal2)
            self.line7.set_xdata(self.x_tiempo)
            self.line7.set_ydata(self.y_qdes3)
            self.line8.set_xdata(self.x_tiempo)
            self.line8.set_ydata(self.y_qreal3)
            self.line10.set_xdata(self.x_tiempo)
            self.line10.set_ydata(self.y_qdes4)
            self.line11.set_xdata(self.x_tiempo)
            self.line11.set_ydata(self.y_qreal4)

            self.ax1.relim()
            self.ax1.autoscale_view()
            self.ax3.relim()
            self.ax3.autoscale_view()
            self.ax5.relim()
            self.ax5.autoscale_view()
            self.ax7.relim()
            self.ax7.autoscale_view()       
            for ax in [self.ax1, self.ax3, self.ax5]:
                ax.set_xticklabels([])
            plt.pause(0.001)

    def run(self):
        while not rospy.is_shutdown():
            self.plot_graph()

        plt.ioff()
        plt.show()


if __name__ == "__main__":
    graficar = Graficar()
    graficar.run()



