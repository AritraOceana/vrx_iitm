#!/usr/bin/env python3

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def callback(data):
    # xyz = ros_numpy.point_cloud2.get_xyz_points(data, remove_nans=True)
    xyz = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for p in xyz:
        pc_list.append( [p[0],p[1],p[2]] )
    pc_list = np.array(p)
    np.savetxt('pcl_data.csv',pc_list,delimiter=',')

def receive_message():
  
    # Tells rospy the name of the node.
    rospy.init_node('pointcloud2toxyz', anonymous=True)
    
    # Node is subscribing to the video_frames topic
    rospy.Subscriber('wamv/sensors/lidars/lidar_wamv/points', PointCloud2, callback)
  
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  
    # Close down the video stream

if __name__ == '__main__':
    receive_message()