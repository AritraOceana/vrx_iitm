import rospy  # Python library for ROS
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library

"""
This script was created to just store the images 
and later use them to perform some basic image processing
"""
class save_images():

    def __init__(self):

        # initialise a count
        self.count = 0

        # initialise a image counter
        self.image_count = 0

        # create a subscriber object
        self.image_sub = rospy.Subscriber('wamv/sensors/cameras/front_left_camera/image_raw', Image, self.callback)

    def callback(self, data):

        self.count += 1

        if not self.count % 15 == 0:
            return

        rospy.loginfo(f"count = {self.count}")

        # Used to convert between ROS and OpenCV images
        br = CvBridge()

        # Convert ROS Image message to OpenCV image
        current_frame = br.imgmsg_to_cv2(data, "bgr8")

        filename = f"/home/aditya/vrx_ws/src/VRX_iitm/docking_soln/images/image_{self.image_count}.jpg"

        cv2.imwrite(filename, current_frame)

        self.image_count += 1


if __name__ == '__main__':
    try:
        rospy.init_node('front_left_camera_sub', anonymous=True)
        save_images()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass