#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

global q_ref
q_ref= np.array([0,0])

def function(q_des):    
    global q_ref
    q_ref = q_des.data


if __name__ == "__main__":
    rospy.init_node('adaptative_controller')
    rospy.Subscriber('q_des',Float64MultiArray,function)
    pub = rospy.Publisher('q_ref',Float64MultiArray,queue_size=1)
    rate = rospy.Rate(10) #2 Hz

    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = q_ref
        pub.publish(msg)
        rate.sleep()