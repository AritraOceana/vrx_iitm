#!/usr/bin/env python3

import rospy
import time
import numpy as np
from math import atan2, cos, sin, radians, pi, sqrt, log
import pyproj
from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import GeoPath
from vrx_gazebo.msg import Task
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32

goal_pose_list = []            # List to store the goal poses
goal_pose_seq = []             # List to store the goal poses in the sequence in which to approach
wamv_initial_pose = []         # To store the initial pose
wamv_pose = np.zeros(3)        # To store the current pose
er = np.array([-2.0, -6, -10]) # the global variable er =[e_x, e_y, e_yaw]
er_dot = np.array([1.0, 1, 1]) # time derivative of error
er_int = np.array([0, 0, 0])   # variable to store the er integration
timeout = False                # timeout or not
remaining_time = 300           # remaining time for the task
task_name = None

delta = 10                     # LOS guidance ( lookahead distance )


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


# Euclidean distance between current pose and goal pose
def euclidean_dist(my_pose, curr_goal_pose):
    dist = sqrt((my_pose[0]-curr_goal_pose[0])**2 + (my_pose[1]-curr_goal_pose[1])**2)
    return dist


# Crosstrack error from current pose to the line joining the consecutive goals
def crosstrack(my_pose, line_coeff):
    a = line_coeff[0]
    b = line_coeff[1]
    c = line_coeff[2]

    x = my_pose[0]
    y = my_pose[1]
    dist = (a*x+b*y+c)/sqrt(a**2 + b**2)
    return dist


# Function to bring the range of yaw error range in -pi to pi
def change_range(angle):
    if -2*pi<=angle<=-pi:
        return angle+2*pi
    elif pi<angle<=2*pi:
        return angle-2*pi
    else:
        return angle


# Function to calculate pose error according to the given formula
def get_pose_error(my_pose, curr_goal_pose):
    d = euclidean_dist(my_pose, curr_goal_pose)
    h = abs(change_range(my_pose[2] - curr_goal_pose[2]))
    E = d+0.75*h
    return E


# Function to get the sequence in which to visit the goals based on their proximity to previous goal
def update_seq(pose_list, initial_pose):
    pose_seq = []
    pose_seq.append(initial_pose)
    for i in range(len(pose_list)-1):
      remaining_nodes_list = [item for item in pose_list if item not in pose_seq]
      distances_list = np.zeros(len(remaining_nodes_list))
      for j in range(len(remaining_nodes_list)):
          distances_list[j] = (remaining_nodes_list[j][0]-pose_seq[-1][0])**2 + (remaining_nodes_list[j][1]-pose_seq[-1][1])**2
      k = np.argmin(distances_list)
      pose_seq.append(remaining_nodes_list[k])
    last_node = [item for item in pose_list if item not in pose_seq]
    pose_seq.append(last_node[0])
    return pose_seq


# Function to get x,y coordinates of goal from GPS coordinates
def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
# Calculate distance and azimuth between GPS points
    geodesic = pyproj.Geod(ellps='WGS84')
    azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)

# Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
# Convert azimuth to radians
    azimuth = radians(azimuth)
    y = adjacent = cos(azimuth) * distance
    x = opposite = sin(azimuth) * distance
    # print(x)
    # print(y)
    # rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))
    return x, y


# Callback function for getting the waypoint poses
def goal_path_sub_callback(msg):
    origin_lat = -33.73
    origin_long = 150.67

    # if  297 <= remaining_time <= 299:
    for i in range(0,len(msg.poses)):
        global goal_pose_list
        lat = msg.poses[i].pose.position.latitude
        lon = msg.poses[i].pose.position.longitude
        x, y = calc_goal(origin_lat, origin_long, lat, lon)
        orientation_q = msg.poses[i].pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        pose = [x, y, yaw]
        goal_pose_list.append(pose)


# Callback function to get timeout value and time remaining until the task is completed
def task_timeout_callback(msg):
    global timeout
    global remaining_time
    global task_name
    timeout = msg.timed_out
    task_name = msg.name
    remaining_time = msg.remaining_time.to_sec()


# Callback function to get pose error value	
def pose_error_callback(msg):
	global pose_error
	pose_error = msg.data


# Callback function to get the pose of wamv
# from the localization node and update the global error variables
def odom_filtered_callback(msg):
    global wamv_initial_pose
    global goal_pose_list
    global remaining_time
    global goal_pose_seq
    global wamv_pose

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    if  299 >= remaining_time > 297:
        x_initial = msg.pose.pose.position.x
        y_initial = msg.pose.pose.position.y
        yaw_initial = yaw
        wamv_initial_pose = [x_initial, y_initial, yaw_initial]
        goal_pose_seq = update_seq(goal_pose_list, wamv_initial_pose)

    wamv_pose[0] = msg.pose.pose.position.x
    wamv_pose[1] = msg.pose.pose.position.y
    wamv_pose[2] = yaw

    # er[0] = msg.pose.pose.position.x - goal_pose[0]
    # er[1] = msg.pose.pose.position.y - goal_pose[1]
    # er[2] = change_range(yaw - goal_pose[2])
    er_dot[0] = msg.twist.twist.linear.x
    er_dot[1] = msg.twist.twist.linear.y
    er_dot[2] = msg.twist.twist.angular.z


if __name__ == '__main__':
    rospy.init_node('wayfindng_solution_node')
    # Subscribing to all important topics
    task_time_sub = rospy.Subscriber("/vrx/task/info",Task,task_timeout_callback)
    rate = rospy.Rate(10)
    time.sleep(10)

    goal_sub = rospy.Subscriber("/vrx/wayfinding/waypoints",GeoPath,goal_path_sub_callback)
    #pose_error_sub = rospy.Subscriber("/vrx/station_keeping/pose_error",Float64,pose_error_callback)
    wamv_odom_filtered = rospy.Subscriber("/wamv/robot_localization/odometry/filtered",Odometry,odom_filtered_callback)
        
    # Publishers for the thrusters
    pub_l_cmd = rospy.Publisher("/wamv/thrusters/left_thrust_cmd", Float32, queue_size = 10)
    pub_r_cmd = rospy.Publisher("/wamv/thrusters/right_thrust_cmd", Float32, queue_size = 10)
    pub_lat_cmd = rospy.Publisher("/wamv/thrusters/lateral_thrust_cmd", Float32, queue_size = 10)
        
    msg_l = Float32()
    msg_r = Float32()
    msg_lat = Float32()
    
    while timeout == False:
        if remaining_time <= 297:
            # print(goal_pose_seq)
            for i in range(1, len(goal_pose_seq)):

                # get the current goal
                current_goal = goal_pose_seq[i]
                print(current_goal)
                er_int = 0

                error_threshold = 0.4

                if i == len(goal_pose_seq)-1:
                    error_threshold = 0

                # Initiallize the minimum goal pose error variable
                min_current_goal_pose_error = get_pose_error(wamv_pose, current_goal)

                timer = 0

                # Keep targeting the same goal till the pose error for it drops below 0.12
                while min_current_goal_pose_error > error_threshold:
                # while min_current_goal_pose_error > 1.2:
                    
                    print(min_current_goal_pose_error)

                    current_goal_pose_error = get_pose_error(wamv_pose, current_goal)
                    if current_goal_pose_error < min_current_goal_pose_error:
                        min_current_goal_pose_error = current_goal_pose_error

                    if i == 1:
                        timer_start = remaining_time
                    if 12 < current_goal_pose_error < 13:
                        timer_start = remaining_time

                    print(timer)
                    print('\n')

                    # Perform station keeping if within 10 m from the goal else LOS guidance
                    # Get the error values for either cases
                    if euclidean_dist(wamv_pose, current_goal) < 11:
                        # print("Station_keeping")
                        timer = timer_start-remaining_time

                        if timer > 40:
                            min_current_goal_pose_error = 0.1

                        er[0] = wamv_pose[0] - current_goal[0]
                        er[1] = wamv_pose[1] - current_goal[1]
                        er[2] = change_range(wamv_pose[2] - current_goal[2])

                        # integral error in case of station keeping only
                        er_int = er_int + (er)*(1/15)

                        Kp = -np.array([[300, 0, 0],[0, 300, 0],[0, 0, 200]])       # Proportional gain
                        Kd = np.array([[80, 0, 0],[0, 80, 0],[0, 0, -50]])           # Derivative gains
                        Ki = -np.array([[0, 0, 0],[0, 0, 0],[0, 0, 1]])           # Integral 
                        
                        tau1 = Kp.dot(er) + Kd.dot(er_dot) + Ki.dot(er_int)

                        # heading angle
                        psi = wamv_pose[2]
                        
                        # # Needs some changes (correct relation between body-fixed frame thrusts and applied thrusts)
                        # # Rotation matrix ( rotation about z by angle psi )
                        rot_mat = np.array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])
                        # Thrust alocation to Body frame
                        # t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])
                        t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])
        
                        # Resultant matrix
                        res_mat = rot_mat.dot(t_mat)
                        res_mat_inv = np.linalg.inv(res_mat)
                        
                        # Calculation of actual thrusts to provide
                        tau2 = res_mat_inv.dot(tau1)

                    else:
                        # print("LOS")
                        # Get the required geometrical values for LOS guidance
                        x_0 = goal_pose_seq[i-1][0]
                        y_0 = goal_pose_seq[i-1][1]
                        x_1 = goal_pose_seq[i][0]
                        y_1 = goal_pose_seq[i][1]
                        alpha = atan2(y_1-y_0,x_1-x_0)
                        line_coeff = [y_1-y_0, -(x_1-x_0), x_1*y_0-x_0*y_1]
                        e = crosstrack(wamv_pose, line_coeff)

                        psi_d = atan2(goal_pose_seq[i][1]-goal_pose_seq[i-1][1], goal_pose_seq[i][0]-goal_pose_seq[i-1][0])
                        # psi_d = atan2(y_los-wamv_pose[1], x_los-wamv_pose[0])
                        # psi_d = atan2(y_1-y_0, x_1-x_0)
                        
                        Kp = np.array([[70, 0, 0],[0, 200, 0],[0, 0, 500]])       # Proportional gain
                        Kd = np.array([[20, 0, 0],[0, 50, 0],[0, 0, 10]])           # Derivative gains
                        
                        er_los = np.array([0.0, 0, 0])
                        er_los_dot = np.array([0.0, 0, 0])
                        er_los[0] = delta
                        er_los[1] = e
                        er_los[2] = change_range(psi_d-wamv_pose[2])
                        
                        v = er_dot[1]
                        u = er_dot[0]
                        chi = atan2(v,u)
                        U = sqrt(u**2 + v**2)
                        theta = (pi/2) - chi + alpha
                        
                        er_los_dot[0] = U*cos(theta)
                        er_los_dot[1] = U*sin(theta)
                        er_los_dot[2] = er_dot[2]

                        # heading angle
                        psi = wamv_pose[2] - alpha
                        
                        # # Needs some changes (correct relation between body-fixed frame thrusts and applied thrusts)
                        # # Rotation matrix ( rotation about z by angle psi )
                        rot_mat = np.array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])
                        # Thrust alocation to Body frame
                        t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])
        
                        # Resultant matrix
                        res_mat = rot_mat.dot(t_mat)
                        res_mat_inv = np.linalg.inv(res_mat)

                        tau1 = Kp.dot(er_los) + Kd.dot(er_los_dot)
                    
                        # Calculation of actual thrusts to provide
                        tau2 = res_mat_inv.dot(tau1)
                    
                    msg_l = inverse_glf_map(tau2[0])
                    msg_r = inverse_glf_map(tau2[1])
                    msg_lat = inverse_glf_map(tau2[2])
                    
                    pub_l_cmd.publish(msg_l)
                    pub_r_cmd.publish(msg_r)
                    pub_lat_cmd.publish(msg_lat)
                    rate.sleep()
                
                print("Goal "+str(i)+" achieved!")
                if i == len(goal_pose_seq)-1:
                    print("All waypoints reached!")
                    exit()



