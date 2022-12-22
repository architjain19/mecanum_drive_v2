#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64, Float64
from geometry_msgs.msg import Twist

pub_cmd = None

wheel_lf = 0.0
wheel_rf = 0.0
wheel_lr = 0.0
wheel_rr = 0.0

vel_x = 0.0
vel_y = 0.0
vel_z = 0.0

def forward_kinematics():

    wheel_sep_width = 0.271
    wheel_sep_length = 0.220
    wheel_radius = 0.0639

    # linear.x = (wheel_front_left + wheel_front_right + wheel_rear_left + wheel_rear_right) * (WHEEL_RADIUS/4)
    # linear.y = ( -wheel_front_left + wheel_front_right + wheel_rear_left – wheel_rear_right) * (WHEEL_RADIUS/4)
    # angular.z = ( -wheel_front_left + wheel_front_right – wheel_rear_left + wheel_rear_right) * (WHEEL_RADIUS/(4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)))

    vel = Twist()

    vel_x = (wheel_lf + wheel_rf + wheel_lr + wheel_rr) * (wheel_radius/4) * 55
    vel_y = (-wheel_lf + wheel_rf - wheel_lr + wheel_rr) * (wheel_radius/4) * 55
    vel_z = (-wheel_lf + wheel_rf + wheel_lr - wheel_rr) * (wheel_radius/(4 * (wheel_sep_width + wheel_sep_length))) * 200

    vel.linear.x = vel_x
    vel.linear.y = vel_y*-1
    vel.angular.z = vel_z

    pub_cmd.publish(vel)

    print("\n\nLinear X: ", vel_x, "\nLinear Y: ", vel_y, "\nAngular Z: ", vel_z)


def jlf_cb(msg):
    global wheel_lf
    wheel_lf = msg.data
    # print("lf: ", wheel_lf)
def jrf_cb(msg):
    global wheel_rf
    wheel_rf = msg.data
    # print("rf: ", wheel_rf)
def jlr_cb(msg):
    global wheel_lr
    wheel_lr = msg.data
    # print("lf: ", wheel_lr)
def jrr_cb(msg):
    global wheel_rr
    wheel_rr = msg.data
    # print("rr: ", wheel_rr)


def main():
    
    global pub_cmd

    rospy.init_node('fk_map')

    rospy.Subscriber('/lf_pos', Float64, callback=jlf_cb)
    rospy.Subscriber('/rf_pos', Float64, callback=jrf_cb)
    rospy.Subscriber('/lr_pos', Float64, callback=jlr_cb)
    rospy.Subscriber('/rr_pos', Float64, callback=jrr_cb)

    pub_cmd = rospy.Publisher('/mecanum_drive_v2/cmd_vel', Twist, queue_size=1)

    # rospy.spin()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        forward_kinematics()

        rate.sleep()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
















































# VERSION - 1

# #!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Int64, Float64
# from geometry_msgs.msg import Twist

# pub_cmd = None

# wheel_lf = 0.0
# wheel_rf = 0.0
# wheel_lr = 0.0
# wheel_rr = 0.0

# vel_x = 0.0
# vel_y = 0.0
# vel_z = 0.0

# def inverse_kinematics():

#     wheel_sep_width = 0.271
#     wheel_sep_length = 0.220
#     wheel_radius = 0.0639

#     # linear.x = (wheel_front_left + wheel_front_right + wheel_rear_left + wheel_rear_right) * (WHEEL_RADIUS/4)
#     # linear.y = ( -wheel_front_left + wheel_front_right + wheel_rear_left – wheel_rear_right) * (WHEEL_RADIUS/4)
#     # angular.z = ( -wheel_front_left + wheel_front_right – wheel_rear_left + wheel_rear_right) * (WHEEL_RADIUS/(4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)))

#     vel = Twist()

#     vel_x = (wheel_lf + wheel_rf + wheel_lr + wheel_rr) * (wheel_radius/4) * 100
#     vel_y = (-wheel_lf + wheel_rf - wheel_lr + wheel_rr) * (wheel_radius/4) * 100
#     vel_z = (-wheel_lf + wheel_rf + wheel_lr - wheel_rr) * (wheel_radius/(4 * (wheel_sep_width + wheel_sep_length))) * 200

#     vel.linear.x = vel_x
#     vel.linear.y = vel_y
#     vel.angular.z = vel_z

#     pub_cmd.publish(vel)

#     print("\n\nLinear X: ", vel_x, "\nLinear Y: ", vel_y, "\nAngular Z: ", vel_z)


# def jlf_cb(msg):
#     global wheel_lf
#     wheel_lf = msg.data
#     # print("lf: ", wheel_lf)
# def jrf_cb(msg):
#     global wheel_rf
#     wheel_rf = msg.data
#     # print("rf: ", wheel_rf)
# def jlr_cb(msg):
#     global wheel_lr
#     wheel_lr = msg.data
#     # print("lf: ", wheel_lr)
# def jrr_cb(msg):
#     global wheel_rr
#     wheel_rr = msg.data
#     # print("rr: ", wheel_rr)


# def main():
    
#     global pub_cmd

#     rospy.init_node('fk_map')

#     rospy.Subscriber('/lf_pos', Int64, callback=jlf_cb)
#     rospy.Subscriber('/rf_pos', Int64, callback=jrf_cb)
#     rospy.Subscriber('/lr_pos', Int64, callback=jlr_cb)
#     rospy.Subscriber('/rr_pos', Int64, callback=jrr_cb)

#     pub_cmd = rospy.Publisher('/mecanum_drive_v2/cmd_vel', Twist, queue_size=1)

#     # rospy.spin()
#     rate = rospy.Rate(100)

#     while not rospy.is_shutdown():

#         inverse_kinematics()

#         rate.sleep()



# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass