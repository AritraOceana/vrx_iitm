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
from usv_msgs.msg import RangeBearing
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
from functions import inverse_glf_map


class acoustic_pinger_tracking_with_obstacle_avoidance():

    def __init__(self):
        self.pinger_loc = np.zeros(3)                     # Acoustic sensor data [range, bearing, elevation]
        self.wamv_pose = np.zeros(3)                      # To store the current pose
        self.twist = np.zeros(3)                          # time derivative of error
        self.timeout = False                              # timeout or not
        self.remaining_time = 300                         # remaining time for the task
        self.delta = 10                                   # LOS guidance ( lookahead distance )
        self.obstacle_list = []                           # List to store [x,y] coordinates of nearby obstacles
        self.er_int = np.zeros(3)                         # Integral error required for station keeping at the pinger_loc

        self.msg_l = Float32()
        self.msg_r = Float32()
        self.msg_lat = Float32()

        # Subscribers
        self.task_time_sub = \
            rospy.Subscriber("/vrx/task/info",Task,self.task_timeout_callback)
        self.goal_sub = \
            rospy.Subscriber("/wamv/sensors/pingers/pinger/range_bearing",RangeBearing,self.pinger_sub_callback)
        self.wamv_odom_filtered = \
            rospy.Subscriber("/wamv/robot_localization/odometry/filtered",Odometry,self.odom_filtered_callback)

        # Publishers for thrusters
        self.pub_l_cmd = rospy.Publisher("/wamv/thrusters/left_thrust_cmd", Float32, queue_size = 10)
        self.pub_r_cmd = rospy.Publisher("/wamv/thrusters/right_thrust_cmd", Float32, queue_size = 10)
        self.pub_lat_cmd = rospy.Publisher("/wamv/thrusters/lateral_thrust_cmd", Float32, queue_size = 10)
        self.rate = rospy.Rate(10)

    def pinger_sub_callback(self, msg):
        '''
        Callback function for getting the range, bearing, elevation from the acoustic sensor
        '''
        # origin_lat = -33.73
        # origin_long = 150.67
        self.pinger_loc[0] = msg.range
        self.pinger_loc[1] = msg.bearing
        self.pinger_loc[2] = msg.elevation

    def task_timeout_callback(self, msg):
        '''
        Callback function to get timeout value and time remaining until the task is finished
        '''
        self.timeout = msg.timed_out
        self.remaining_time = msg.remaining_time.to_sec()

    def odom_filtered_callback(self, msg):
        '''
        Callback function to get the pose of wamv
        from the localization node and update the global error variables
        '''
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        self.wamv_pose[0] = msg.pose.pose.position.x
        self.wamv_pose[1] = msg.pose.pose.position.y
        self.wamv_pose[2] = yaw

        self.twist[0] = msg.twist.twist.linear.x
        self.twist[1] = msg.twist.twist.linear.y
        self.twist[2] = msg.twist.twist.angular.z

    def station_keeping(self):
        '''
        This function generates the thrust commands needed to be published to thruster topics
        for station keeping
        '''
        er = np.zeros(3)
        er_dot = np.zeros(3)
        r = self.pinger_loc[0]
        b = self.pinger_loc[1]
        psi = self.wamv_pose[2]

        er[0] = r*cos(b)*cos(psi) - r*sin(b)*sin(psi)
        er[1] = r*cos(b)*sin(psi) - r*sin(b)*cos(psi)
        er[2] = 0
        er_dot[0] = self.twist[0]
        er_dot[1] = self.twist[1]
        er_dot[2] = self.twist[2]

        self.er_int = self.er_int + (er)*(1/15)

        Kp = np.array([[300, 0, 0],[0, 300, 0],[0, 0, 200]])         # Proportional gain
        Kd = -np.array([[120, 0, 0],[0, 120, 0],[0, 0, -50]])        # Derivative gains
        Ki = np.array([[0.01, 0, 0],[0.01, 0, 0],[0, 0, 1]])         # Integral gains

        tau_pid = Kp.dot(er) + Kd.dot(er_dot) + Ki.dot(self.er_int)  # PID controller output

        t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])         # Thrust alocation to Body frame
        rot_mat = np.array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])
        res_mat = rot_mat.dot(t_mat)
        res_mat_inv = np.linalg.inv(res_mat)
        tau_final = res_mat_inv.dot(tau_pid)                         # Calculation of actual thrusts to provide

        r_left = inverse_glf_map(tau_final[0])
        r_right = inverse_glf_map(tau_final[1])
        lateral = inverse_glf_map(tau_final[2])

        return r_left, r_right, lateral

    def obstacle_field_navigation(self):
        '''
        This function generates the thrust commands needed to be published to thruster topics
        for navigating through the obstacle field
        '''
        zeta = 1
        eta = 100
        safe_dist = 8
        grid_size = 21
        
        attractive_potential = \
            lambda x, y: (1/2)*zeta*((x-self.pinger_loc[0])**2 + (y-self.pinger_loc[1])**2)
        repulsive_potential = \
            lambda x, y, x_obs, y_obs: (1/2)*eta*(1/sqrt((x-x_obs)**2 + (y-y_obs)**2)-(1/safe_dist))**2

        h_min, h_max = -grid_size, grid_size
        v_min, v_max = -grid_size, grid_size

        xv, yv = np.meshgrid(np.arange(h_min, h_max, 0.5),
                     np.arange(v_min, v_max, 0.5))
        
        net_repulsive_potential = np.zeros((4*grid_size,4*grid_size))

        for obs_num in range(len(self.obstacle_list)):
            for i in range(4*grid_size):
                for j in range(4*grid_size):
                    net_repulsive_potential[i,j] += \
                        repulsive_potential(xv[0,i],yv[j,0],self.obstacle_list[obs_num][0],self.obstacle_list[obs_num][1])

        result_matrix = net_repulsive_potential + attractive_potential(xv, yv)
        yd, xd = np.gradient(-result_matrix)
        alpha = atan2(yd[2*grid_size,2*grid_size],xd[2*grid_size,2*grid_size])
        er = np.zeros(3)
        er_dot = np.zeros(3)

        r = self.delta
        b = alpha
        psi = self.wamv_pose[2]

        er[0] = r*cos(b)*cos(psi) - r*sin(b)*sin(psi)
        er[1] = r*cos(b)*sin(psi) - r*sin(b)*cos(psi)
        er[2] = alpha
        er_dot[0] = self.twist[0]
        er_dot[1] = self.twist[1]
        er_dot[2] = self.twist[2]

        Kp = np.array([[70, 0, 0],[0, 70, 0],[0, 0, 250]])          # Proportional gain
        Kd = np.array([[20, 0, 0],[0, 50, 0],[0, 0, 10]])           # Derivative gains

        tau_pid = Kp.dot(er) + Kd.dot(er_dot)

        t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])        # Thrust alocation to Body frame
        rot_mat = np.array([[cos(psi), -sin(psi), 0], [sin(psi), cos(psi), 0], [0, 0, 1]])
        res_mat = rot_mat.dot(t_mat)
        res_mat_inv = np.linalg.inv(res_mat)
        tau_final = res_mat_inv.dot(tau_pid)                        # Calculation of actual thrusts to provide

        r_left = inverse_glf_map(tau_final[0])
        r_right = inverse_glf_map(tau_final[1])
        lateral = inverse_glf_map(tau_final[2])

        return r_left, r_right, lateral

    def execute(self):
        msg_l = Float32()
        msg_r = Float32()
        msg_lat = Float32()
        while self.timeout == False:
            if self.pinger_loc[0] > 10:
                msg_l,msg_r,msg_lat = self.obstacle_field_navigation()
                self.pub_l_cmd.publish(msg_l)
                self.pub_r_cmd.publish(msg_r)
                self.pub_lat_cmd.publish(msg_lat)
                self.rate.sleep()
            else:
                msg_l,msg_r,msg_lat = self.station_keeping()
                self.pub_l_cmd.publish(msg_l)
                self.pub_r_cmd.publish(msg_r)
                self.pub_lat_cmd.publish(msg_lat)
                self.rate.sleep()

    

#####################################################################################################################

''' 
Types of goals

IN gate : 1
OUT gate : 3
MID gate : 2
station_keeping : 0
wamv_initial_pose : 4
NOT gate : 6

'''

######################################################################################################################


if __name__ == '__main__':
    rospy.init_node('gymkhana_solution_node')
