# Running mecanum robot using linear and angular velocities by pubshing them on cmd_vel topic and converting individual wheel rpm using inverse kinematics

# Gazebo Plugin - Object Controller



# RUN 29/11/2022 - Mecanum_Drive_V2
- gazebo.launch
    - object_controller -> cmd_vel, odom, base_footprint
- rviz.launch
    - fixed = map or odom any one will work
- ros_sync_trial_1.py
    - subscribe = odom coordinates
    - publish = cmd_vel velocities
    - publish = arduino indivdual velocities using custom range function
- rosrun rosserial_python serial_node.py /dev/ttyACM0
    - Sending data from ROS to Arduino
    - VEL_TO_RPM_APPROACH.ino in Arduino
- slam_gmapping.launch
    - To create a map in rviz and publish to map topic
- Variables
    - kP_linear = 8.0
    - kP_angular = 6.0
    - range_from = 0, 1000
    - range_to = 0, 37
- TF transformation
    - map -> odom = 0 0 0 0 0 0



## Gmapping process
- sudo apt-get install ros-noetic-slam-gmapping [Install respective packages needed for slam]
- rosrun gmapping slam_gmapping scan:=/mecanum_drive_v2/scan [To create map use this]
- rosrun map_server map_saver -f my_world_map [To save map use this]


# Note:
- And run other launch for gazebo and rviz accordingly.
- keep the fixed frame to map in rviz... launch rviz using 'rosrun rviz rviz'
