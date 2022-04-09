#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs import point_cloud2
from sklearn.cluster import KMeans
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

"""
This script creates a node that will subscribe to the /voxel_filtered_cloud topic
and then calculate the goal pose away from the scan_code_buoy to station_keep and 
publish it to /scan_code_coordinates topic as latitude, longitude

Next the station keeping node will read the message on the topic /scan_code_coordinates
and perform station_keeping
"""

class filter_cloud_and_publish():

    def __init__(self):
        self.scan_code_coords = []      # list of (x, y) of the scan code buoy in the lidar frame
        self.x_set = None
        self.y_set = None
        self.theta_buoy = None
        self.away_from_buoy = 5         # distance to station keep away from the buoy
        self.angular_limit = np.pi/4    # setting the angular range of the filter

        self.msg = GeoPoseStamped()     # initialising the message to be published
        
        self.points_subscriber = \
            rospy.Subscriber("/voxel_filtered_cloud", PointCloud2, self.lidar_callback) # subscribe to point cloud data
        self.set_point_publisher =  \
            rospy.Publisher("/scan_code_coordinates", GeoPoseStamped, queue_size=10) # publish set point as geo stamped coordinates
        
        
    def ssa(self, angle):
        """ 
        returns a smallest signed angle between (-pi, pi) \n
        expected input `angle` -> degrees \n
        expected output `angle` -> degrees
        """
        angle = angle % (360)
        if angle > 180:
            angle = angle - 360
        return angle    #*math.pi/180

    def llh_from_ned(self, l0, mu0, xn, yn):
        """ 
        l0  --> latitude of the origin \n
        mu0 --> longitude of the origin \n
        xn  --> x-coordinate of the point to be transformed \n
        yn  --> y-coordinate of the point to be transformed \n

        returns the latitude, longitude from (x,y), given the origin (l0,m0)
        """
        re = 6378137    # radius of earth
        e = 0.0818      # eccentricity
        Rn = re / math.sqrt(1 - e**2 * (math.sin(mu0))**2)
        Rm =  Rn * (1 - e**2) / math.sqrt(1 - e**2 * (math.sin(mu0))**2)

        delta_l = yn * math.atan2(1, Rm*math.cos(mu0))
        delta_mu = xn * math.atan2(1, Rn)

        l = self.ssa(l0 + delta_l)
        mu = self.ssa(mu0 + delta_mu)

        return l , mu

    def navsat_filtered_callback(self, data):
            """
            recieve gps location of wamv \n
            and publish the gps location of the point (5m away from buoy) to stattion keep \n
            """
            wamv_lat_init = data.latitude
            wamv_long_init = data.longitude
    
            lat_set, long_set = self.llh_from_ned(wamv_lat_init, wamv_long_init, self.x_set, self.y_set)
            rospy.loginfo(f"lat = {lat_set}\t long = {long_set}")
    
            
            self.msg.pose.position.latitude = lat_set
            self.msg.pose.position.longitude = long_set

            self.set_point_publisher.publish(self.msg)

    def lidar_callback(self, data):
            """
            recieve lidar data from /voxel filtered topic
            and filter it further
            """

            # recieve lidar points
            xyz = np.array(list(point_cloud2.read_points(data,field_names = ("x", "y", "z"), skip_nans=True)))
            rospy.loginfo(len(xyz))

            theta = np.arctan2(xyz[:,1],xyz[:,0])

            # allow only the points inside the angular limit set here
            ind = np.logical_and(theta>-self.angular_limit, theta<self.angular_limit)
            xyz = xyz[ind,:]

            if len(xyz) != 0:
                # perform KMeans on the data
                kmeans = KMeans(n_clusters = 1).fit(xyz)
                cluster_center = kmeans.cluster_centers_
                self.scan_code_coords.append(cluster_center)
                
            
            if len(self.scan_code_coords) == 10:
                # collect 10 coordinates and then average it to get the centroid of the cloud
                scan_code_coordinates = [sum(x) / len(x) for x in zip(*self.scan_code_coords)]
                scan_code_coord_x = scan_code_coordinates[0][0]
                scan_code_coord_y = scan_code_coordinates[0][1] 
    
                # find the point 5 m away from the above found point
                r_buoy = math.sqrt(scan_code_coord_x**2 + scan_code_coord_y**2)
                theta_buoy = math.atan2(scan_code_coord_y, scan_code_coord_x)
    
                self.x_set = (r_buoy - self.away_from_buoy) * math.cos(theta_buoy)
                self.y_set = (r_buoy - self.away_from_buoy) * math.sin(theta_buoy) 
                rospy.loginfo(f"x = {self.x_set}\t y = {self.y_set}\t theta = {theta_buoy}")

                # populate the orientation part of the msg defined earlier
                # q = quaternion_from_euler(0, 0, theta_buoy)

                # self.msg.pose.orientation.x = q[0]
                # self.msg.pose.orientation.y = q[1]
                # self.msg.pose.orientation.z = q[2]
                # self.msg.pose.orientation.w = q[3]

                # recieve gps message from the topic
                wamv_geo_loc_msg = rospy.wait_for_message('/wamv/robot_localization/gps/filtered', NavSatFix, timeout=None)
                wamv_odom_filtered_msg = rospy.wait_for_message("/wamv/robot_localization/odometry/filtered", Odometry, timeout=None)

                wamv_orientation_q = wamv_odom_filtered_msg.pose.pose.orientation
                orientation_list = [wamv_orientation_q.x, wamv_orientation_q.y, wamv_orientation_q.z, wamv_orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
                theta_global = yaw + theta_buoy

                q = quaternion_from_euler(0, 0, theta_global)

                self.msg.pose.orientation.x = q[0]
                self.msg.pose.orientation.y = q[1]
                self.msg.pose.orientation.z = q[2]
                self.msg.pose.orientation.w = q[3]

                # publish the gps coordinates 
                self.navsat_filtered_callback(wamv_geo_loc_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('filter_and_publish_geo')
        filter_cloud_and_publish()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass