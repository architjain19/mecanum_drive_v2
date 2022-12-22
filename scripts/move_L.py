#!/usr/bin/env python3

import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseArray

import time

hola_x = 0.0
hola_y = 0.0
hola_theta = 0.0

def odometryCb(msg):
    global hola_x, hola_y, hola_theta

    # Write your code to take the msg and update the three variables    
    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (hola_roll, hola_pitch, hola_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # print("\n\nHola X: ", hola_x, "\nHola Y: ", hola_y, "\nHola Theta: ", hola_theta)



def main():

    rospy.init_node('mecanum_drive_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    rospy.Subscriber('/odom', Odometry, callback=odometryCb)

    rate = rospy.Rate(100)

    vel = Twist()

    vel_x = 0.0
    vel_y = 0.0
    vel_z = 0.0

    while not rospy.is_shutdown():

        # print("\n\nHola X: ", hola_x, "\nHola Y: ", hola_y, "\nHola Theta: ", hola_theta)

        while(-0.05 <= hola_x <= 1.0 and -0.05 <= hola_y <= 0.05):
            vel.linear.x = 1.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            pub.publish(vel)

        print("REACHED POSITION - 1")

        while(-0.95 <= hola_x <= 1.05 and -0.05 <= hola_y <= 1.0):
            vel.linear.x = 0.0
            vel.linear.y = 1.0
            vel.angular.z = 0.0
            pub.publish(vel)

        print("REACHED POSITION - 2")

        while(True):
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            pub.publish(vel)
        

        rate.sleep()




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass