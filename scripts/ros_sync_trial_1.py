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

from std_msgs.msg import Float64

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

def ranging(old_number):
    old_minimum, old_maximum = 0, 1000
    new_minimum, new_maximum = 0, 37
    # old_number = 3

    old_length = old_maximum - old_minimum
    new_length = new_maximum - new_minimum

    new_number = (((old_number - old_minimum) * new_length) / old_length) + new_minimum

    return int(new_number)



def inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, vel_x, vel_y, vel_z):

    wheel_front_left = (1/wheel_radius)*(vel_x - vel_y - (wheel_sep_width + wheel_sep_length) * vel_z)
    wheel_front_right = (1/wheel_radius)*(vel_x + vel_y + (wheel_sep_width + wheel_sep_length) * vel_z)
    wheel_rear_left = (1/wheel_radius)*(vel_x + vel_y - (wheel_sep_width + wheel_sep_length) * vel_z)
    wheel_rear_right = (1/wheel_radius)*(vel_x - vel_y + (wheel_sep_width + wheel_sep_length) * vel_z)

    wheel_front_right = -1*wheel_front_right
    wheel_rear_right = -1*wheel_rear_right

    # 1 rad/s = 9.549297 RPM

    s_fl = wheel_front_left * 9.549297
    s_fr = wheel_front_right * 9.549297
    s_rl = wheel_rear_left * 9.549297
    s_rr = wheel_rear_right * 9.549297

    s_fl = ranging(s_fl)
    s_fr = ranging(s_fr)
    s_rl = ranging(s_rl)
    s_rr = ranging(s_rr)

    if s_fl > 43:
        s_fl = 43
    if s_fl < -43:
        s_fl = -43
    if s_fr > 43:
        s_fr = 43
    if s_fr < -43:
        s_fr = -43
    if s_rl > 43:
        s_rl = 43
    if s_rl < -43:
        s_rl = -43
    if s_rr > 43:
        s_rr = 43
    if s_rr < -43:
        s_rr = -43

    # print("\nLinear: Vel X - ", vel_x, "\nLinear: Vel Y - ", vel_y, "\nAngular: Vel Z - ", vel_z)
    print("\nWheel: Front Left - ", s_fl, "\nWheel: Front Right - ", s_fr, "\nWheel: Rear Left - ", s_rl, "\nWheel: Rear Right - ", s_rr)

    pub_lf.data = s_fl
    pub_arduino_lf.publish(pub_lf)

    pub_rf.data = s_fr
    pub_arduino_rf.publish(pub_rf)

    pub_lr.data = s_rl
    pub_arduino_lr.publish(pub_lr)

    pub_rr.data = s_rr
    pub_arduino_rr.publish(pub_rr)
    # publish these messages



def main():
    global pub_arduino_lf, pub_arduino_rf, pub_arduino_lr, pub_arduino_rr
    global pub_lf, pub_rf, pub_lr, pub_rr
    rospy.init_node('ros_sync_trial_1')
    
    pub = rospy.Publisher('/mecanum_drive_v2/cmd_vel', Twist, queue_size = 1)
    pub_arduino_lf = rospy.Publisher('/lf_pos', Float64, queue_size = 1)
    pub_arduino_rf = rospy.Publisher('/rf_pos', Float64, queue_size = 1)
    pub_arduino_lr = rospy.Publisher('/lr_pos', Float64, queue_size = 1)
    pub_arduino_rr = rospy.Publisher('/rr_pos', Float64, queue_size = 1)
    rospy.Subscriber('/mecanum_drive_v2/odom', Odometry, callback=odometryCb)

    rate = rospy.Rate(200)

    vel = Twist()
    pub_lf = Float64()
    pub_rf = Float64()
    pub_lr = Float64()
    pub_rr = Float64()

    vel_x = 0.0
    vel_y = 0.0
    vel_z = 0.0

    # # SQUARE
    # x_goals = [0.5, 0.5, 0, 0]
    # y_goals = [0, 0.5, 0.5, 0]

    x_goals = [-2]
    y_goals = [0]

    kP_linear = 8.0
    kP_angular = 6.0

    count = 0

    wheel_sep_width = 0.271
    wheel_sep_length = 0.220

    wheel_radius = 0.0639

    while not rospy.is_shutdown():

        # print("\n\nHola X: ", hola_x, "\nHola Y: ", hola_y, "\nHola Theta: ", hola_theta)
        x_d = x_goals[count]
        y_d = y_goals[count]
        
        error_x = x_d - hola_x
        error_y = y_d - hola_y

        angle_d = math.atan2(error_y, error_x)
        error_hypot = math.sqrt((error_x ** 2) + (error_y ** 2))

        error_dist_x = math.cos(angle_d)*error_hypot
        error_dist_y = math.sin(angle_d)*error_hypot

        vel_x = error_dist_x * kP_linear
        vel_y = error_dist_y * kP_linear       

        # print(error_dist_x, error_dist_y)

        if ((abs(error_x) <= 0.025) and (abs(error_y) <= 0.025)):
            vel_x = 0.0
            vel_y = 0.0
            vel_z = 0.0
            vel.linear.x = vel_x
            vel.linear.y = vel_y
            vel.angular.z = vel_z
            pub.publish(vel)
            inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, vel_x, vel_y, vel_z)

            while( (hola_theta < -0.025) or (hola_theta > 0.025) ):
                vel_x = 0.0
                vel_y = 0.0
                vel_z = hola_theta * kP_angular
                vel.linear.x = vel_x
                vel.linear.y = vel_y
                vel.angular.z = vel_z
                pub.publish(vel)
                inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, vel_x, vel_y, vel_z)

                rate.sleep()
            
            vel_x = 0.0
            vel_y = 0.0
            vel_z = 0.0
            vel.linear.x = vel_x
            vel.linear.y = vel_y
            vel.angular.z = vel_z
            pub.publish(vel)
            inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, vel_x, vel_y, vel_z)

            print("Reached Goal: ", count+1)

            if count < len(x_goals)-1:
                count+=1
            else:
                print("---------FINISHED---------")
                break

        vel.linear.x = vel_x
        vel.linear.y = vel_y
        vel.angular.z = vel_z
        pub.publish(vel)
        inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, vel_x, vel_y, vel_z)

        rate.sleep()




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass