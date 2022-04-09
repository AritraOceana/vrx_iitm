from math import log, sqrt, pi, radians, sin, cos
import pyproj

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

# Along track error from current pose to the line joining the consecutive goals
def along_track(my_pose, first, second):
    along_track_error = ((my_pose[0:2]-second[0:2]).dot(second[0:2]-first[0:2]))/euclidean_dist(first, second)
    return along_track_error

# Crosstrack error from current pose to the line joining the consecutive goals
def crosstrack(my_pose, first_pose, second_pose):
    x_0 = first_pose[0]
    y_0 = first_pose[1]
    x_1 = second_pose[0]
    y_1 = second_pose[1]

    a = y_1-y_0
    b = -(x_1-x_0)
    c = x_1*y_0-x_0*y_1

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

# Function to calculate pose error according to the given formula
def get_pose_error(my_pose, curr_goal_pose):
    d = euclidean_dist(my_pose, curr_goal_pose)
    h = abs(change_range(my_pose[2] - curr_goal_pose[2]))
    E = d+0.75*h
    return E