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

pub_lf = None
pub_rf = None
pub_lr = None
pub_rr = None

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

from time import time, sleep
prevMilli = 0
milliseconds = 0


def inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, vel_x, vel_y, vel_z):

    if(vel_x > 0):
        vel_x /= 50
    elif(vel_x < 0):
        vel_x /= 55

    if(vel_y > 0):
        vel_y /= 40
    elif(vel_y < 0):
        vel_y /= 43

    # vel_y /= 40
    vel_z /= 55

    wheel_front_left = (1/wheel_radius)*(vel_x - vel_y - (wheel_sep_width + wheel_sep_length) * vel_z)
    wheel_front_right = (1/wheel_radius)*(vel_x + vel_y + (wheel_sep_width + wheel_sep_length) * vel_z)
    wheel_rear_left = (1/wheel_radius)*(vel_x + vel_y - (wheel_sep_width + wheel_sep_length) * vel_z)
    wheel_rear_right = (1/wheel_radius)*(vel_x - vel_y + (wheel_sep_width + wheel_sep_length) * vel_z)

    wheel_front_right = -1*wheel_front_right
    wheel_rear_right = -1*wheel_rear_right

    # 1 rad/s = 9.549297 RPM

    # s_fl = wheel_front_left * 9.549297
    # s_fr = wheel_front_right * 9.549297
    # s_rl = wheel_rear_left * 9.549297
    # s_rr = wheel_rear_right * 9.549297

    # print("\nLinear: Vel X - ", vel_x, "\nLinear: Vel Y - ", vel_y, "\nAngular: Vel Z - ", vel_z)
    # print("\nWheel: Front Left - ", wheel_front_left, "\nWheel: Front Right - ", wheel_front_right, "\nWheel: Rear Left - ", wheel_rear_left, "\nWheel: Rear Right - ", wheel_rear_right)


    encoder_lf = Float64()
    encoder_rf = Float64()
    encoder_lr = Float64()
    encoder_rr = Float64()

    global prevMilli
    milliseconds = int(time() * 1000)
    # print("Time in milliseconds since epoch", milliseconds)

    if (milliseconds - prevMilli) > 30:
        prevMilli = milliseconds

        encoder_lf.data = (wheel_front_left * 356) / (1 * 60) * (255 / 92)
        encoder_rf.data = (wheel_front_right * 356) / (1 * 60) * (255 / 92)
        encoder_lr.data = (wheel_rear_left * 356) / (1 * 60) * (255 / 92)
        encoder_rr.data = (wheel_rear_right * 356) / (1 * 60) * (255 / 92)

        # wheel_front_left = 0.0
        # wheel_front_right = 0.0
        # wheel_rear_left = 0.0
        # wheel_rear_right = 0.0
        
        print("\nEncoder: Front Left - ", encoder_lf.data, "\nEncoder: Front Right - ", encoder_rf.data, "\nEncoder: Rear Left - ", encoder_lr.data, "\nEncoder: Rear Right - ", encoder_rr.data)
        

    pub_lf.publish(encoder_lf)
    pub_rf.publish(encoder_rf)
    pub_lr.publish(encoder_lr)
    pub_rr.publish(encoder_rr)


def main():
    global pub_lf, pub_lr, pub_rf, pub_rr

    rospy.init_node('inv_kin_controller')
    
    pub = rospy.Publisher('/mecanum_drive_v2/cmd_vel', Twist, queue_size = 1)
    pub_lf = rospy.Publisher('lf_pos', Float64, queue_size = 1)
    pub_lr = rospy.Publisher('lr_pos', Float64, queue_size = 1)
    pub_rf = rospy.Publisher('rf_pos', Float64, queue_size = 1)
    pub_rr = rospy.Publisher('rr_pos', Float64, queue_size = 1)
    rospy.Subscriber('/mecanum_drive_v2/odom', Odometry, callback=odometryCb)

    rate = rospy.Rate(100)

    vel = Twist()

    vel_x = 0.0
    vel_y = 0.0
    vel_z = 0.0

    x_goals = [-1.1, -1.1, -1.1, 0.5, 0.5, 0]
    y_goals = [0, -0.25, 0, 0, 0.5, 0]
    # x_goals = [1, 1, 0]
    # y_goals = [0, 0.5, 0]
    # x_goals = [0, 0]
    # y_goals = [1, 0]

    kP_linear = 5.5
    kP_angular = 5.5

    count = 0

    wheel_sep_width = 0.271
    wheel_sep_length = 0.220

    wheel_radius = 0.0639

    sleep(1)

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

        if(error_dist_x > 0.05):
            vel_x = kP_linear
        elif(error_dist_x < -0.05):
            vel_x = -kP_linear
        else:
            vel_x = 0.0
        
        if(error_dist_y > 0.05):
            vel_y = kP_linear
        elif(error_dist_y < -0.05):
            vel_y = -kP_linear
        else:
            vel_y = 0.0
        

        print(error_dist_x, error_dist_y)

        if ((abs(error_x) <= 0.05) and (abs(error_y) <= 0.05)):
            vel_x = 0.0
            vel_y = 0.0
            vel_z = 0.0

            vel.linear.x = vel_x
            vel.linear.y = vel_y
            vel.angular.z = vel_z
            pub.publish(vel)
            inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, vel_x, vel_y, vel_z)

            while( (hola_theta < -0.05) or (hola_theta > 0.05) ):
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
                sleep(1)
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




















# #!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Int64, Float64
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion

# pub_cmd = None
# pub_lf = None
# pub_rf = None
# pub_lr = None
# pub_rr = None

# wheel_front_left = 0.0
# wheel_front_right = 0.0
# wheel_rear_left = 0.0
# wheel_rear_right = 0.0

# vel_x = 0.0
# vel_y = 0.0
# vel_z = 0.0

# meca_x = 0.0
# meca_y = 0.0
# meca_theta = 0.0

# wheel_sep_width = 0.271
# wheel_sep_length = 0.220
# wheel_radius = 0.0639



# encoder_lf = Float64()
# encoder_rf = Float64()
# encoder_lr = Float64()
# encoder_rr = Float64()

# encoder_lf.data = 0.0
# encoder_rf.data = 0.0
# encoder_lr.data = 0.0
# encoder_rr.data = 0.0

# from time import time
# prevMilli = 0
# milliseconds = 0

# def send_data():
#     global prevMilli, wheel_front_left, wheel_front_right, wheel_rear_left, wheel_rear_right

#     milliseconds = int(time() * 1000)
#     # print("Time in milliseconds since epoch", milliseconds)

#     if (milliseconds - prevMilli) > 500:
#         prevMilli = milliseconds

#         encoder_lf.data = (wheel_front_left * 356) / (0.10471975512 * 60)
#         encoder_rf.data = (wheel_front_right * 356) / (0.10471975512 * 60)
#         encoder_lr.data = (wheel_rear_left * 356) / (0.10471975512 * 60)        
#         encoder_rr.data = (wheel_rear_right * 356) / (0.10471975512 * 60)

#         wheel_front_left = 0.0
#         wheel_front_right = 0.0
#         wheel_rear_left = 0.0
#         wheel_rear_right = 0.0

#     pub_lf.publish(encoder_lf)
#     pub_rf.publish(encoder_rf)
#     pub_lr.publish(encoder_lr)
#     pub_rr.publish(encoder_rr)


 
# def odometryCb(msg):
#     global meca_x, meca_y, meca_theta

#     meca_x = msg.pose.pose.position.x
#     meca_y = msg.pose.pose.position.y
#     rot_q = msg.pose.pose.orientation
#     (meca_roll, meca_pitch, meca_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
#     print("\n\nMeca X: ", meca_x, "\nMeca Y: ", meca_y, "\nMeca Theta: ", meca_theta)
#     # send_data()


# def inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, vel_x, vel_y, vel_z):

#     global wheel_front_left, wheel_front_right, wheel_rear_left, wheel_rear_right

#     # vel_x /=55
#     # vel_y /=55
#     # vel_z /=200

#     wheel_front_left = (1/wheel_radius)*(vel_x - vel_y - (wheel_sep_width + wheel_sep_length) * vel_z)
#     wheel_front_right = (1/wheel_radius)*(vel_x + vel_y + (wheel_sep_width + wheel_sep_length) * vel_z)
#     wheel_rear_left = (1/wheel_radius)*(vel_x + vel_y - (wheel_sep_width + wheel_sep_length) * vel_z)
#     wheel_rear_right = (1/wheel_radius)*(vel_x - vel_y + (wheel_sep_width + wheel_sep_length) * vel_z)

#     wheel_front_right = -1*wheel_front_right
#     wheel_rear_right = -1*wheel_rear_right

#     # 1 rad/s = 9.549297 RPM
#     # wheel_front_left = wheel_front_left * 9.549297
#     # wheel_front_right = wheel_front_right * 9.549297
#     # wheel_rear_left = wheel_rear_left * 9.549297
#     # wheel_rear_right = wheel_rear_right * 9.549297

#     # print("\nLinear: Vel X - ", vel_x, "\nLinear: Vel Y - ", vel_y, "\nAngular: Vel Z - ", vel_z)
#     # print("\nWheel: Front Left - ", wheel_front_left, "\nWheel: Front Right - ", wheel_front_right, "\nWheel: Rear Left - ", wheel_rear_left, "\nWheel: Rear Right - ", wheel_rear_right)



# def main():
    
#     global pub_cmd, pub_lf, pub_rf, pub_lr, pub_rr
#     global vel_x, vel_y, vel_z

#     rospy.init_node('ik_map')

#     pub_lf = rospy.Publisher('/lf_pos', Float64, queue_size=1)
#     pub_rf = rospy.Publisher('/rf_pos', Float64, queue_size=1)
#     pub_lr = rospy.Publisher('/lr_pos', Float64, queue_size=1)
#     pub_rr = rospy.Publisher('/rr_pos', Float64, queue_size=1)

#     pub_cmd = rospy.Publisher('/mecanum_drive_v2/cmd_vel', Twist, queue_size=1)
#     rospy.Subscriber('/mecanum_drive_v2/odom', Odometry, callback=odometryCb)

#     # rospy.spin()
#     rate = rospy.Rate(100)

#     vel = Twist()
#     vel_x = 0.0
#     vel_y = 0.0
#     vel_z = 0.0

#     x_d = 1.0
#     y_d = 0.0

#     kP = 6.0

#     while not rospy.is_shutdown():
        
#         error_x = x_d - meca_x
#         error_y = y_d - meca_y

#         print(error_x, error_y)

#         if( abs(error_x) < 0.03 ):
            
#             vel_x = 0.0
#             vel_y = 0.0
#             vel_z = 0.0
#             vel.linear.x = vel_x
#             vel.linear.y = vel_y
#             vel.angular.z = vel_z
#             pub_cmd.publish(vel)
#             print("Reached !")
#             inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, vel_x, vel_y, vel_z)

        
#         vel_x = kP
#         vel_y = 0
#         vel_z = 0
#         vel.linear.x = vel_x
#         vel.linear.y = vel_y
#         vel.angular.z = vel_z
#         pub_cmd.publish(vel)
#         inverse_kinematics(wheel_radius, wheel_sep_width, wheel_sep_length, vel_x, vel_y, vel_z)
    
#         rate.sleep()



# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass