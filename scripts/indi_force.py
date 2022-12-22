#!/usr/bin/env python3

from turtle import left
import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Wrench, Vector3
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion


from geometry_msgs.msg import PoseArray

import time

import numpy as np


def odometryCb(msg):
    global hola_x, hola_y, hola_theta

    # Write your code to take the msg and update the three variables    
    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (hola_roll, hola_pitch, hola_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # print("\n\nHola X: ", hola_x, "\nHola Y: ", hola_y, "\nHola Theta: ", hola_theta)

def main():

    rospy.init_node('mecanum_controller')
    
    pubfl = rospy.Publisher('/frontL_wheel_force', Wrench, queue_size = 1)
    pubfr = rospy.Publisher('/frontR_wheel_force', Wrench, queue_size = 1)
    pubbr = rospy.Publisher('/backR_wheel_force', Wrench, queue_size = 1)
    pubbl = rospy.Publisher('/backL_wheel_force', Wrench, queue_size = 1)
    rospy.Subscriber('/odom', Odometry, callback=odometryCb)

    rate = rospy.Rate(100)

    vel_x, vel_y, vel_z = 100, 100, 0    


    radius_of_wheel = 0.0639
    l_invKin = 0.22 
    w_invKin = 0.271

    h_01 = ([(-l_invKin-w_invKin), 1, -1], [(l_invKin+w_invKin), 1, 1], [(l_invKin+w_invKin), 1, -1], [(-l_invKin-w_invKin), 1, 1])

    # h_01 = ([1, -0.5, -0.5], [0, 0.866, -0.866], [1, 1, 1])
    # h_01 = np.linalg.inv(h_01)

    wrenchFL = Wrench()
    wrenchFR = Wrench()
    wrenchBR = Wrench()
    wrenchBL = Wrench()

    while not rospy.is_shutdown():

        h_02 = ([vel_x], [vel_y], [vel_z])
        result = np.dot(h_01, h_02)
        
        wrenchFL.force.x, wrenchFR.force.x, wrenchBR.force.x, wrenchBL.force.x = result[0][0]/radius_of_wheel, result[1][0]/radius_of_wheel, result[2][0]/radius_of_wheel, result[3][0]/radius_of_wheel
        pubfl.publish(wrenchFL)
        pubfr.publish(wrenchFR)
        pubbr.publish(wrenchBR)
        pubbl.publish(wrenchBL)

        print(wrenchFL)
        print(wrenchFR)
        print(wrenchBR)
        print(wrenchBL)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass