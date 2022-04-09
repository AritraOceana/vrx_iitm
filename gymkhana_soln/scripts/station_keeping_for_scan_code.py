#!/usr/bin/env python3

import rospy
import numpy as np
import math
import pyproj
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32

"""
This script creates a node that subscribes the /scan_code_coordinates topic 
and then perform station keeping to the desired location.

We want to station keep at the goal till the sequence of the color is being read.
Once we read the color sequence, next task would be to move towards the docking bay.

Currently the problem is that the station keeping is malfunctioning. It is wierdly station keeping to some other point.
"""

class setpoint_tracking():

    def __init__(self):
        self.goal_pose = np.array([0.0, 0, 0])	                            # The x,y and yaw angle of the goal
        self.er = np.array([-2.0, -6, -10])                                 # the global variable er =[e_x, e_y, e_yaw]
        self.er_dot = np.array([1.0, 1, 1])                                 # time derivative of error
        self.er_int = np.array([0, 0, 0])                                   # variable to store the er integration
        self.timeout = False                                                # timeout or not
        self.pose_error = 0.0                                               # didn't use. maybe useful for just checking?
        self.tau = np.array([[1],[1],[1]])                                  # torques/moments in x,y and yaw

        self.Kp = -np.array([[1000, 0, 0],[0, 1000, 0],[0, 0, 1000]])       # Proportional gain
        self.Kd = np.array([[50, 0, 0],[0, 50, 0],[0, 0, -50]])             # Derivative gains
        self.Ki = -np.array([[0, 0, 0],[0, 0, 0],[0, 0, 1]])                # Integral gains

        self.wamv_odom_filtered = rospy.Subscriber("/wamv/robot_localization/odometry/filtered", Odometry, self.odom_filtered_callback)

        # Publishers for the thrusters
        self.pub_l_cmd = rospy.Publisher("/wamv/thrusters/left_thrust_cmd", Float32, queue_size = 10)
        self.pub_r_cmd = rospy.Publisher("/wamv/thrusters/right_thrust_cmd", Float32, queue_size = 10)
        self.pub_lat_cmd = rospy.Publisher("/wamv/thrusters/lateral_thrust_cmd", Float32, queue_size = 10)
        self.rate = rospy.Rate(10)


    def inverse_glf_map(self, T):
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

        self.cmd = M - (1/B)*math.log((((K-A)/(T-A))**nu)-C)

        return self.cmd

    def calc_goal(self, origin_lat, origin_long, goal_lat, goal_long):
        """
        Calculate distance and azimuth between GPS points
        """
        geodesic = pyproj.Geod(ellps='WGS84')
        azimuth,back_azimuth,distance = geodesic.inv(origin_long, origin_lat, goal_long, goal_lat)
        azimuth = math.radians(azimuth)
        self.y = adjacent = math.cos(azimuth) * distance
        self.x = opposite = math.sin(azimuth) * distance
        rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(self.x, self.y))
        return self.x, self.y

    def goal_sub_callback(self, msg):
        origin_lat = -33.73
        origin_long = 150.67
        goal_lat = msg.pose.position.latitude
        goal_long = msg.pose.position.longitude
        x, y = self.calc_goal(origin_lat, origin_long, goal_lat, goal_long)
        self.goal_pose[0] = x
        self.goal_pose[1] = y
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.goal_pose[2] = yaw

    def change_range(self, angle):
        if -2*math.pi <= angle <= -math.pi:
            return angle + 2*math.pi
        else:
            return angle

    def odom_filtered_callback(self, msg):
        """
        Callback function to get the pose of wamv
        from the localization node and update the global error variables
        """
        self.er[0] = msg.pose.pose.position.x - self.goal_pose[0]
        self.er[1] = msg.pose.pose.position.y - self.goal_pose[1]
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.er[2] = self.change_range(yaw - self.goal_pose[2])

        self.er_dot[0] = msg.twist.twist.linear.x
        self.er_dot[1] = msg.twist.twist.linear.y
        self.er_dot[2] = msg.twist.twist.angular.z

    def execute(self):
        
        # self.goal_sub = rospy.Subscriber("/scan_code_coordinates", GeoPoseStamped, self.goal_sub_callback)
        rospy.loginfo("waiting for the coordinates of the  scan code buoy")
        goal_msg = rospy.wait_for_message('/scan_code_coordinates', GeoPoseStamped, timeout=None)
        self.goal_sub_callback(goal_msg)
        

        msg_l = Float32()
        msg_r = Float32()
        msg_lat = Float32()

        while True:
            rospy.loginfo(self.er)

            # heading angle
            psi = self.er[2] + self.goal_pose[2]

            # Rotation matrix ( rotation about z by angle phi )
            rot_mat = np.array([[math.cos(psi), -math.sin(psi), 0], [math.sin(psi), math.cos(psi), 0], [0, 0, 1]])
            # Thrust alocation to Body frame
            t_mat = np.array([[1, 1, 0], [0, 0, 1], [-1, 1, 0]])

            # Resultant matrix
            res_mat = rot_mat.dot(t_mat)
            res_mat_inv = np.linalg.inv(res_mat)

            # integral error
            self.er_int = self.er_int + (self.er)*(1/15)

	    # PID controller output
            tau1 = self.Kp.dot(self.er) + self.Kd.dot(self.er_dot) + self.Ki.dot(self.er_int)

	    # Calculation of actual thrusts to provide
            tau2 = res_mat_inv.dot(tau1)     
            msg_l = self.inverse_glf_map(tau2[0])
            msg_r = self.inverse_glf_map(tau2[1])
            msg_lat = self.inverse_glf_map(tau2[2])

            self.pub_l_cmd.publish(msg_l)
            self.pub_r_cmd.publish(msg_r)
            self.pub_lat_cmd.publish(msg_lat)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('scan_code_tracking')
        track_scan_code = setpoint_tracking()
        track_scan_code.execute()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass