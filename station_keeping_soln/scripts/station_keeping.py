#!/usr/bin/env python3

import rospy
import time
import numpy as np
from math import atan2, cos, sin, radians, pi, sqrt, log
import pyproj
from geographic_msgs.msg import GeoPoseStamped
from vrx_gazebo.msg import Task
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32


goal_pose = np.array([0.0, 0, 0])	# The x,y and yaw angle of the goal
er = np.array([-2.0, -6, -10])      # the global variable er =[e_x, e_y, e_yaw]
er_dot = np.array([1.0, 1, 1])      # time derivative of error
er_int = np.array([0, 0, 0])        # variable to store the er integration
timeout = False                     # timeout or not
pose_error = 0.0                    # didn't use. maybe useful for just checking?
tau = np.array([[1],[1],[1]])       # torques/moments in x,y and yaw
task_name = None

# Kp = -np.array([[2.5, 0, 0],[0, 2.5, 0],[0, 0, 2]])       # Proportional gain
# Kd = np.array([[5, 0, 0],[0, 5, 0],[0, 0, -5]])           # Derivative gains
# Ki = -np.array([[0, 0, 0],[0, 0, 0],[0, 0, 1]])           # Integral gains


Kp = -np.array([[1000, 0, 0],[0, 1000, 0],[0, 0, 1000]])       # Proportional gain
Kd = np.array([[50, 0, 0],[0, 50, 0],[0, 0, -50]])           # Derivative gains
Ki = -np.array([[0.01, 0, 0],[0, 0.01, 0],[0, 0, 1]])           # Integral gains


# Function to map the thrust to the commands in range(-1,1)
def inverse_glf_map(T):
    if T >= 250:
        T = 250
    elif T < -100:
        T = -100
    if 0.06 <= T < 1.2:
        T = 0.06
    
    if T >= 1.2:
        A = 0.01
        K = 59.82
        B = 5.0
        nu = 0.38
        C = 0.56
        M = 0.

    if T <= 0.06:
        A = -199.13
        K = -0.09
        B = 8.84
        nu = 5.34
        C = 0.99
        M = -0.57
    cmd = M - (1/B)*log((((K-A)/(T-A))**nu)-C)
    return cmd


# Function to get x,y coordinates of goal from GPS coordinates
def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
# Calculate distance and azimuth between GPS points
    geodesic = pyproj.Geod(ellps='WGS84')
    azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)

# Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lengths of a right-angle triangle
# Convert azimuth to radians
    azimuth = radians(azimuth)
    y = adjacent = cos(azimuth) * distance
    x = opposite = sin(azimuth) * distance
    print(x)
    print(y)
    rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))
    return x, y


# Callback function for getting the goal pose
def goal_sub_callback(msg):
    global goal_pose
    origin_lat = -33.73
    origin_long = 150.67
    goal_lat = msg.pose.position.latitude
    goal_long = msg.pose.position.longitude
    x, y = calc_goal(origin_lat, origin_long, goal_lat, goal_long)
    goal_pose[0] = x
    goal_pose[1] = y
    orientation_q = msg.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    goal_pose[2] = yaw


# Callback function to get timeout value 
def task_timeout_callback(msg):
    global timeout
    global task_name
    task_name = msg.name
    timeout = msg.timed_out
	

# Callback function to get pose error value
def pose_error_callback(msg):
	global pose_error
	pose_error = msg.data
	

# Function to bring the range of yaw error range in -pi to pi
def change_range(angle):
    if -2*pi<=angle<=-pi:
        return angle+2*pi
    elif pi<angle<=2*pi:
        return angle-2*pi
    else:
        return angle


# Callback function to get the pose of wamv
# from the localization node and update the global error variables
def odom_filtered_callback(msg):
    global er
    global er_dot
    global er_int
    global er_prev
    er[0] = msg.pose.pose.position.x - goal_pose[0]
    er[1] = msg.pose.pose.position.y - goal_pose[1]
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    er[2] = change_range(yaw - goal_pose[2])
    
    er_dot[0] = msg.twist.twist.linear.x
    er_dot[1] = msg.twist.twist.linear.y
    er_dot[2] = msg.twist.twist.angular.z


if __name__ == '__main__':
    rospy.init_node('station_keeping_node')
    # Subscribing to all important topics

    task_time_sub = rospy.Subscriber("/vrx/task/info",Task,task_timeout_callback)
    rate = rospy.Rate(10)
    time.sleep(10)

    goal_sub = rospy.Subscriber("/vrx/station_keeping/goal",GeoPoseStamped,goal_sub_callback)
    pose_error_sub = rospy.Subscriber("/vrx/station_keeping/pose_error",Float64,pose_error_callback)
    wamv_odom_filtered = rospy.Subscriber("/wamv/robot_localization/odometry/filtered",Odometry,odom_filtered_callback)
        
    # Publishers for the thrusters
    pub_l_cmd = rospy.Publisher("/wamv/thrusters/left_thrust_cmd", Float32, queue_size = 10)
    pub_r_cmd = rospy.Publisher("/wamv/thrusters/right_thrust_cmd", Float32, queue_size = 10)
    pub_lat_cmd = rospy.Publisher("/wamv/thrusters/lateral_thrust_cmd", Float32, queue_size = 10)
        
    msg_l = Float32()
    msg_r = Float32()
    msg_lat = Float32()
    
    while timeout == False:

        # heading angle
        psi = er[2] + goal_pose[2]

        # # Needs some changes (correct relation between body-fixed frame thrusts and applied thrusts)
        # Rotation matrix ( rotation about z by angle phi )
        rot_mat = np.array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])
        # Thrust alocation to Body frame
        t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])

        # Resultant matrix
        res_mat = rot_mat.dot(t_mat)
        res_mat_inv = np.linalg.inv(res_mat)

        # integral error
        er_int = er_int + (er)*(1/15)

    # PID controller output
        tau1 = Kp.dot(er) + Kd.dot(er_dot) + Ki.dot(er_int)

    # Calculation of actual thrusts to provide
        tau2 = res_mat_inv.dot(tau1)
        # tau = tau2/(np.linalg.norm(tau2))         
        msg_l = inverse_glf_map(tau2[0])
        msg_r = inverse_glf_map(tau2[1])
        msg_lat = inverse_glf_map(tau2[2])

        # msg_l = tau[0]
        # msg_r = tau[1]
        # msg_lat = tau[2]
        pub_l_cmd.publish(msg_l)
        pub_r_cmd.publish(msg_r)
        pub_lat_cmd.publish(msg_lat)
        rate.sleep()
