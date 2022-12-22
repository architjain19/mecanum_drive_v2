#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64, Float64

pub_lf = None
pub_rf = None
pub_lr = None
pub_rr = None



def jlf_cb(msg):
    wheel_pos = 0
    if (msg.data >= 0):
        wheel_pos = 5
    elif (msg.data < 0):
        wheel_pos = -5
    
    



    pos = Float64()
    pos.data = wheel_pos
    pub_lf.publish(pos)
    print("lf: ", pos)
def jrf_cb(msg):
    wheel_pos = msg.data * 6.28/356
    # print(msg)
    pos = Float64()
    pos.data = wheel_pos
    pub_rf.publish(pos)
    print("rf: ", pos)
def jlr_cb(msg):
    wheel_pos = msg.data * (6.28/356) * -1
    # print(msg)
    pos = Float64()
    pos.data = wheel_pos
    pub_lr.publish(pos)
    print("lr: ", pos)
def jrr_cb(msg):
    wheel_pos = msg.data * 6.28/356
    # print(msg)
    pos = Float64()
    pos.data = wheel_pos
    pub_rr.publish(pos)
    print("rr: ", pos)


def main():
    
    global pub_lf, pub_rf, pub_lr, pub_rr

    rospy.init_node('position_trial')

    pub_lf = rospy.Publisher('/mecanum_drive_v2/jleft_front_wheel_position_controller/command', Float64, queue_size=1)
    pub_rf = rospy.Publisher('/mecanum_drive_v2/jright_front_wheel_position_controller/command', Float64, queue_size=1)
    pub_lr = rospy.Publisher('/mecanum_drive_v2/jleft_rear_wheel_position_controller/command', Float64, queue_size=1)
    pub_rr = rospy.Publisher('/mecanum_drive_v2/jright_rear_wheel_position_controller/command', Float64, queue_size=1)


    rospy.Subscriber('/lf_pos', Int64, callback=jlf_cb)
    rospy.Subscriber('/rf_pos', Int64, callback=jrf_cb)
    rospy.Subscriber('/lr_pos', Int64, callback=jlr_cb)
    rospy.Subscriber('/rr_pos', Int64, callback=jrr_cb)

    rospy.spin()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass