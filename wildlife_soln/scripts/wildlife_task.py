#!/usr/bin/env python3

import rospy
import numpy as np
from math import atan2, cos, sin, pi, sqrt
from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import GeoPath
from vrx_gazebo.msg import Task
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
from functions import inverse_glf_map, euclidean_dist, along_track, crosstrack, change_range, calc_goal, get_pose_error

goal_pose_list = []            # List to store the goal poses
goal_pose_seq = []             # List to store the goal poses in the sequence in which to approach
obstacles_list = []            # List to store the obstacle poses
wamv_initial_pose = []         # To store the initial pose
wamv_pose = np.zeros(3)        # To store the current pose
obstacle_avoided = False


er = np.array([-2.0, -6, -10]) # the global variable er =[e_x, e_y, e_yaw]
twist = np.array([1.0, 1, 1])  # time derivative of error
er_int = np.array([0, 0, 0])   # variable to store the er integration


timeout = False                # timeout or not
remaining_time = 300           # remaining time for the task

delta = 10                     # LOS guidance ( lookahead distance )

#####################################################################################################################

''' Types of poses

platypus : 1
turtle : 2
crocodile : 3
station_keeping (near animals) : 0
wamv_initial_pose : 4
crocodile_avoidance : 5

'''

######################################################################################################################


# Function to get the sequence in which to visit the goals based on their proximity to previous goal
def update_seq(pose_list, initial_pose):
    pose_seq = []
    station_poses = []
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
    
    for i in range(len(pose_seq)-1):
        x_0 = pose_seq[i][0]
        y_0 = pose_seq[i][1]
        x_1 = pose_seq[i+1][0]
        y_1 = pose_seq[i+1][1]
        if (x_1-x_0)**2 + (y_1-y_0)**2 > 225:
            alpha = atan2(y_1-y_0,x_1-x_0)
            x_s = x_1 - 13*cos(alpha)
            y_s = y_1 - 13*sin(alpha)
            pose = [x_s, y_s, alpha, 0]
        else:
            pose = [x_1,y_1,alpha,0]
        station_poses.append(pose)

    new_pose_seq = []
    new_pose_seq.append(initial_pose)
    for i in range(len(pose_seq)-1):
        new_pose_seq.append(station_poses[i])
        new_pose_seq.append(pose_seq[i+1])

    return new_pose_seq

####################################################################################################################


# Callback function for getting the animal poses
def animals_sub_callback(msg):
    origin_lat = -33.73
    origin_long = 150.67
    global goal_pose_list
    global obstacles_list
    global goal_pose_seq
    global obstacles_list

    goal_list = []
    obs_list = []

    # if  297 <= remaining_time <= 299:
    for i in range(0,len(msg.poses)):
        animal = 0
        animal_type = msg.poses[i].header.frame_id
        if animal_type == "crocodile":
            animal = 3
        elif animal_type == "platypus":
            animal = 1
        elif animal_type == "turtle":
            animal = 2
        lat = msg.poses[i].pose.position.latitude
        lon = msg.poses[i].pose.position.longitude
        x, y = calc_goal(origin_lat, origin_long, lat, lon)
        orientation_q = msg.poses[i].pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        pose = [x, y, yaw, animal]
        if animal != 3:
            goal_list.append(pose)
        elif animal == 3:
            obs_list.append(pose)

    goal_pose_list = goal_list
    obstacles_list = obs_list

    if  295 >= remaining_time > 0:
        goal_pose_seq = update_seq(goal_pose_list, wamv_initial_pose)


# Callback function to get timeout value and time remaining until the task is completed
def task_timeout_callback(msg):
    global timeout
    global remaining_time
    timeout = msg.timed_out
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
        wamv_initial_pose = [x_initial, y_initial, yaw, 4]

    wamv_pose[0] = msg.pose.pose.position.x
    wamv_pose[1] = msg.pose.pose.position.y
    wamv_pose[2] = yaw

    twist[0] = msg.twist.twist.linear.x
    twist[1] = msg.twist.twist.linear.y
    twist[2] = msg.twist.twist.angular.z


def station_keeping(my_pose, goal_pose, vel):
    er = np.array([0.0, 0, 0])
    er_dot = np.array([0.0, 0, 0])

    er[0] = my_pose[0] - goal_pose[0]
    er[1] = my_pose[1] - goal_pose[1]
    er[2] = my_pose[2] - goal_pose[2]
    er_dot[0] = vel[0]
    er_dot[1] = vel[1]
    er_dot[2] = vel[2]

    Kp = -np.array([[300, 0, 0],[0, 300, 0],[0, 0, 200]])       # Proportional gain
    Kd = np.array([[150, 0, 0],[0, 150, 0],[0, 0, -50]])        # Derivative gains

    tau_pid = Kp.dot(er) + Kd.dot(er_dot)                       # PID controller output
    psi = my_pose[2]
    
    # Rotation matrix ( rotation about z by angle phi )
    rot_mat = np.array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])

    t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])        # Thrust alocation to Body frame
    res_mat = rot_mat.dot(t_mat)                                # Resultant matrix
    res_mat_inv = np.linalg.inv(res_mat)
    tau_final = res_mat_inv.dot(tau_pid)                        # Calculation of actual thrusts to provide

    r_left = inverse_glf_map(tau_final[0])
    r_right = inverse_glf_map(tau_final[1])
    lateral = inverse_glf_map(tau_final[2])

    return r_left, r_right, lateral


def over_control(my_pose, goal_pose, vel):
    er = np.array([0.0, 0, 0])
    er_dot = np.array([0.0, 0, 0])

    er[0] = my_pose[0] - goal_pose[0]
    er[1] = my_pose[1] - goal_pose[1]
    er[2] = my_pose[2] - goal_pose[2]
    er_dot[0] = vel[0]
    er_dot[1] = vel[1]
    er_dot[2] = vel[2]

    Kp = -np.array([[65, 0, 0],[0, 65, 0],[0, 0, 150]])       # Proportional gain
    Kd = np.array([[150, 0, 0],[0, 150, 0],[0, 0, -50]])        # Derivative gains

    tau_pid = Kp.dot(er) + Kd.dot(er_dot)                       # PID controller output
    psi = my_pose[2]
    
    # Rotation matrix ( rotation about z by angle phi )
    rot_mat = np.array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])

    t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])        # Thrust alocation to Body frame
    res_mat = rot_mat.dot(t_mat)                                # Resultant matrix
    res_mat_inv = np.linalg.inv(res_mat)
    tau_final = res_mat_inv.dot(tau_pid)                        # Calculation of actual thrusts to provide

    r_left = inverse_glf_map(tau_final[0])
    r_right = inverse_glf_map(tau_final[1])
    lateral = inverse_glf_map(tau_final[2])

    return r_left, r_right, lateral


def line_following(my_pose, first_pose, second_pose, vel):

    # Get the required geometrical parameters
    x_0 = first_pose[0]
    y_0 = first_pose[1]
    x_1 = second_pose[0]
    y_1 = second_pose[1]
    alpha = atan2(y_1-y_0,x_1-x_0)
    e = crosstrack(my_pose, first_pose, second_pose)

    psi_d = atan2(y_1-my_pose[1], x_1-my_pose[0])               # Desired heading

    Kp = np.array([[60, 0, 0],[0, 100, 0],[0, 0, 100]])         # Proportional gain
    Kd = np.array([[150, 0, 0],[0, 80, 0],[0, 0, 10]])           # Derivative gains

    er_los = np.array([0.0, 0, 0])
    er_los_dot = np.array([0.0, 0, 0])
    er_los[0] = delta
    er_los[1] = e
    er_los[2] = change_range(psi_d-my_pose[2])
    v = vel[1]
    u = vel[0]
    chi = atan2(v,u)
    U = sqrt(u**2 + v**2)
    theta = (pi/2) - chi + alpha
    er_los_dot[0] = U*cos(theta)
    er_los_dot[1] = U*sin(theta)
    er_los_dot[2] = vel[2]

    psi = my_pose[2] - alpha                                  # heading angle

    ## Rotation matrix ( rotation about z by angle psi )
    rot_mat = np.array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])

    t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])        # Thrust alocation to Body frame               
    res_mat = rot_mat.dot(t_mat)                                # Resultant matrix
    res_mat_inv = np.linalg.inv(res_mat)
    tau_pd = Kp.dot(er_los) + Kd.dot(er_los_dot)                # PID controller output
    # Calculation of actual thrusts to provide
    tau_final = res_mat_inv.dot(tau_pd)

    r_left = inverse_glf_map(tau_final[0])
    r_right = inverse_glf_map(tau_final[1])
    lateral = inverse_glf_map(tau_final[2])

    return r_left, r_right, lateral


def encountering_maneuver_segment(my_pose, first_pose, second_pose, vel):
    x_0 = first_pose[0]
    y_0 = first_pose[1]
    x_1 = second_pose[0]
    y_1 = second_pose[1]

    alpha = atan2(y_1-y_0,x_1-x_0)
    e = crosstrack(my_pose, first_pose, second_pose)

    psi_d = alpha

    er_los = np.array([0.0, 0, 0])
    er_los_dot = np.array([0.0, 0, 0])
    er_los[0] = delta
    er_los[1] = e
    er_los[2] = change_range(psi_d - my_pose[2])

    if circle_pts == 0:
        er_los[2] = 0

    v = vel[1]
    u = vel[0]
    chi = atan2(v,u)
    U = sqrt(u**2 + v**2)
    theta = (pi/2) - chi + alpha
    er_los_dot[0] = U*cos(theta)
    er_los_dot[1] = U*sin(theta)
    er_los_dot[2] = twist[2]
    
    psi = wamv_pose[2] - alpha                               # heading angle
    Kp = np.array([[35, 0, 0],[0, 55, 0],[0, 0, 250]])       # Proportional gain
    Kd = np.array([[10, 0, 0],[0, 35, 0],[0, 0, 10]])        # Derivative gains
    rot_mat = np.array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])
    
    t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])     # Thrust alocation to Body frame
    
    res_mat = rot_mat.dot(t_mat)                             # Resultant matrix
    res_mat_inv = np.linalg.inv(res_mat)
    tau_pd = Kp.dot(er_los) + Kd.dot(er_los_dot)
    
    tau_final = res_mat_inv.dot(tau_pd)                      # Calculation of actual thrusts to provide
    r_left = inverse_glf_map(tau_final[0])
    r_right = inverse_glf_map(tau_final[1])
    lateral = inverse_glf_map(tau_final[2])

    return r_left, r_right, lateral


def obstacle_avoidance_segment(my_pose, first_pose, second_pose, vel):
    x_0 = first_pose[0]
    y_0 = first_pose[1]
    x_1 = second_pose[0]
    y_1 = second_pose[1]

    alpha = atan2(y_1-y_0,x_1-x_0)
    e = crosstrack(my_pose, first_pose, second_pose)

    psi_d = alpha

    er_los = np.array([0.0, 0, 0])
    er_los_dot = np.array([0.0, 0, 0])
    er_los[0] = delta
    er_los[1] = e
    er_los[2] = change_range(psi_d - my_pose[2])

    if obs_points == 0:
        er_los[2] = 0

    v = vel[1]
    u = vel[0]
    chi = atan2(v,u)
    U = sqrt(u**2 + v**2)
    theta = (pi/2) - chi + alpha
    er_los_dot[0] = U*cos(theta)
    er_los_dot[1] = U*sin(theta)
    er_los_dot[2] = twist[2]
    
    psi = wamv_pose[2] - alpha                               # heading angle
    Kp = np.array([[40, 0, 0],[0, 55, 0],[0, 0, 250]])       # Proportional gain
    Kd = np.array([[10, 0, 0],[0, 35, 0],[0, 0, 10]])        # Derivative gains
    rot_mat = np.array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])
    
    t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])     # Thrust alocation to Body frame
    
    res_mat = rot_mat.dot(t_mat)                             # Resultant matrix
    res_mat_inv = np.linalg.inv(res_mat)
    tau_pd = Kp.dot(er_los) + Kd.dot(er_los_dot)
    
    tau_final = res_mat_inv.dot(tau_pd)                      # Calculation of actual thrusts to provide
    r_left = inverse_glf_map(tau_final[0])
    r_right = inverse_glf_map(tau_final[1])
    lateral = inverse_glf_map(tau_final[2])

    return r_left, r_right, lateral


if __name__ == '__main__':
    rospy.init_node('wildlife_solution_node')
    # Subscribing to all important topics
    task_time_sub = rospy.Subscriber("/vrx/task/info",Task,task_timeout_callback)
    goal_sub = rospy.Subscriber("/vrx/wildlife/animals/poses",GeoPath,animals_sub_callback)
    #pose_error_sub = rospy.Subscriber("/vrx/station_keeping/pose_error",Float64,pose_error_callback)
    wamv_odom_filtered = rospy.Subscriber("/wamv/robot_localization/odometry/filtered",Odometry,odom_filtered_callback)
        
    # Publishers for the thrusters
    pub_l_cmd = rospy.Publisher("/wamv/thrusters/left_thrust_cmd", Float32, queue_size = 10)
    pub_r_cmd = rospy.Publisher("/wamv/thrusters/right_thrust_cmd", Float32, queue_size = 10)
    pub_lat_cmd = rospy.Publisher("/wamv/thrusters/lateral_thrust_cmd", Float32, queue_size = 10)
    rate = rospy.Rate(20)
        
    msg_l = Float32()
    msg_r = Float32()
    msg_lat = Float32()
    
    while timeout == False:
        if remaining_time <= 297:

            for i in range(1, len(goal_pose_seq)):

                print(goal_pose_seq)

                # get the current goal and its type
                current_goal = np.array([0.0, 0, 0])
                current_goal[0] = goal_pose_seq[i][0]
                current_goal[1] = goal_pose_seq[i][1]
                current_goal[2] = goal_pose_seq[i][2]

                goal_type = goal_pose_seq[i][3]
                print(goal_type)

                if goal_type == 0:
                    print("station_keeping_goal active")                
                    while get_pose_error(wamv_pose, current_goal) > 0.8:
                        while euclidean_dist(wamv_pose, current_goal) > 10:

                            obstacle_pose = np.array([0.0, 0, 0])
                            obstacle_pose[0] = obstacles_list[0][0]
                            obstacle_pose[1] = obstacles_list[0][1]
                            obstacle_pose[2] = obstacles_list[0][2]

                            obs_points = 0

                            if abs(crosstrack(obstacle_pose, goal_pose_seq[i-1], goal_pose_seq[i])) < 10:
                                if (euclidean_dist(wamv_pose, obstacle_pose) < 14) and not(obstacle_avoided):

                                    print("Obstacle encountered!!")
                                    print(euclidean_dist(wamv_pose, obstacle_pose))

                                    if crosstrack(obstacle_pose, goal_pose_seq[i-1], goal_pose_seq[i]) >= 0: phi = -pi/6
                                    elif crosstrack(obstacle_pose, goal_pose_seq[i-1], goal_pose_seq[i]) < 0: phi = pi/6
                                    # phi = pi/6
                                    R = np.array([[cos(phi), -sin(phi), 0], [sin(phi), cos(phi), 0], [0, 0, 1]])

                                    while obs_points < 4:

                                        obstacle_pose[0] = obstacles_list[0][0]
                                        obstacle_pose[1] = obstacles_list[0][1]
                                        obstacle_pose[2] = obstacles_list[0][2]

                                        if obs_points == 0:
                                            first_pose = wamv_pose

                                        second_pose = obstacle_pose + R.dot(first_pose-obstacle_pose)
                                        alpha = atan2(second_pose[1] - obstacle_pose[1], second_pose[0] - obstacle_pose[0])
                                        second_pose[0] = obstacle_pose[0] + 15*cos(alpha)
                                        second_pose[1] = obstacle_pose[1] + 15*sin(alpha)

                                        while(along_track(wamv_pose,first_pose,second_pose)>6 or along_track(wamv_pose,first_pose,second_pose)<-6):

                                            msg_l,msg_r,msg_lat = obstacle_avoidance_segment(wamv_pose, first_pose, second_pose, twist)
                                
                                            pub_l_cmd.publish(msg_l)
                                            pub_r_cmd.publish(msg_r)
                                            pub_lat_cmd.publish(msg_lat)
                                            rate.sleep()
                                
                                        obs_points += 1
                                        first_pose = second_pose
                                        print("Circle point crossed!")

                                        if obs_points == 3:
                                            obstacle_avoided = True

                            
                            print("LOS")
                            
                            msg_l, msg_r, msg_lat = line_following(wamv_pose, goal_pose_seq[i-1], goal_pose_seq[i], twist)
                            
                            pub_l_cmd.publish(msg_l)
                            pub_r_cmd.publish(msg_r)
                            pub_lat_cmd.publish(msg_lat)
                            rate.sleep()

                        print("Station keeping")

                        msg_l, msg_r, msg_lat = station_keeping(wamv_pose, current_goal, twist)

                        pub_l_cmd.publish(msg_l)
                        pub_r_cmd.publish(msg_r)
                        pub_lat_cmd.publish(msg_lat)
                        rate.sleep()

                    print("Station keeping done!")


                if goal_type == 1 or goal_type == 2:

                    if goal_type == 1:
                        phi = -pi/15
                    elif goal_type == 2:
                        phi = pi/15

                    R = np.array([[cos(phi), -sin(phi), 0], [sin(phi), cos(phi), 0], [0, 0, 1]])

                    circle_pts = 0
                    max_pts = 37
                    while circle_pts < max_pts:
                    # Get the error values for either cases
                        animal_pose = np.array([0.0, 0, 0])
                        animal_pose[0] = goal_pose_seq[i][0]
                        animal_pose[1] = goal_pose_seq[i][1]
                        animal_pose[2] = goal_pose_seq[i][2]

                        if euclidean_dist(wamv_pose, animal_pose) < 6.5:
                            print("Encountering Maneuver")
                            print('\n')
                            
                            if circle_pts == 0:
                                first_pose = wamv_pose
                                
                            second_pose = animal_pose + R.dot(first_pose-animal_pose)
                            
                            while(along_track(wamv_pose,first_pose,second_pose)>4.5 or along_track(wamv_pose,first_pose,second_pose)<-1):

                                msg_l,msg_r,msg_lat = encountering_maneuver_segment(wamv_pose, first_pose, second_pose, twist)
                                
                                pub_l_cmd.publish(msg_l)
                                pub_r_cmd.publish(msg_r)
                                pub_lat_cmd.publish(msg_lat)
                                rate.sleep()
                                
                            circle_pts += 1
                            first_pose = second_pose
                            print("Circle point crossed!")
                            
                        if 6.5 < euclidean_dist(wamv_pose, animal_pose) < 10:
                            print("Just inside Encountering region")

                            circle_center = np.array([0.0, 0, 0])
                            circle_center[0] = goal_pose_seq[i][0]
                            circle_center[1] = goal_pose_seq[i][1]
                            circle_center[2] = wamv_pose[2]
                        
                            msg_l, msg_r, msg_lat = over_control(wamv_pose, circle_center, twist)

                            pub_l_cmd.publish(msg_l)
                            pub_r_cmd.publish(msg_r)
                            pub_lat_cmd.publish(msg_lat)
                            rate.sleep()

                        if euclidean_dist(wamv_pose, animal_pose) > 10:
                            print("LOS")
                            
                            msg_l, msg_r, msg_lat = line_following(wamv_pose, goal_pose_seq[i-1], goal_pose_seq[i], twist)
                            
                            pub_l_cmd.publish(msg_l)
                            pub_r_cmd.publish(msg_r)
                            pub_lat_cmd.publish(msg_lat)
                            rate.sleep()
                    
                    # if circle_pts >= max_pts-1:
                    #     print("Maneuver completed!!")
                    #     msg_l = 1.0
                    #     msg_r = 1.0
                    #     for num in range(5):
                    #         print(num)
                    #         pub_l_cmd.publish(msg_l)
                    #         pub_r_cmd.publish(msg_r)
                    #         rate.sleep()
                
                print("Goal "+str(i)+" achieved!")
                if i == len(goal_pose_seq)-1:
                    print("All waypoints reached!")
                    exit()

