#!/usr/bin/env python3

import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from geographic_msgs.msg import GeoPoseStamped # GeoposeStamped has the detected class and position
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from vrx_gazebo.msg import Task
import time

class yolo_detector():

  def __init__(self, class_names, config_file_name, weight_file_name):
    self.task_name = None
    self.timeout = None

    # Subscriber to left camera image
    rospy.init_node('perception_node', anonymous=True)
    
    # Node is subscribing to the video_frames topic
    rospy.Subscriber("/vrx/task/info",Task, self.task_timeout_callback)
    time.sleep(10)

    if self.task_name == "perception":

        rospy.loginfo(self.task_name)

         # initialise a count
        self.count = 0

        # Read names of classed from the respective file
        with open(class_names, 'r') as f:
          self.classes = f.read().splitlines()

        # Load the configuration file and trainedweight files
        net = cv2.dnn.readNetFromDarknet(config_file_name, weight_file_name)
        self.model = cv2.dnn_DetectionModel(net)

         # make publisher object
        self.publisher = rospy.Publisher('/vrx/perception/landmark', GeoPoseStamped, queue_size=10)

        # Crete an object of the message type
        self.gps = GeoPoseStamped()

        # Node is subscribing to the video_frames topic
        rospy.Subscriber('wamv/sensors/cameras/front_left_camera/image_raw', Image, self.callback_camera)
      
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    else:
        exit()

  def callback_camera(self, data):

    self.count += 1

    if not self.count%15 == 0:
      return

    # rospy.loginfo(f"count = {self.count}")
  
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    
    #rospy.loginfo("receiving video frame")
    
    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data, "bgr8")

    self.model.setInputParams(scale=1 / 255, size=(416, 416), swapRB=True)
    
    classIds, scores, boxes = self.model.detect(current_frame, confThreshold=0.6, nmsThreshold=0.4)

    for (classId, score, box) in zip(classIds, scores, boxes):
        # print(self.classes[classId[0]])
        # rospy.loginfo(f"Object detected: {self.classes[classId]}")
        # cv2.rectangle(current_frame, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]),
        #               color=(0, 255, 0), thickness=2)
    
        # text = '%s: %.2f' % (self.classes[classId], score)
        # cv2.putText(current_frame, text, (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
        #             color=(255, 255, 255), thickness=2)

        self.gps.header.frame_id = self.classes[classId]

        #display the message on the terminal
        # rospy.loginfo(classId)
        #publish the message to the topic
        self.publisher.publish(self.gps)

    # # Display image
    # cv2.imshow("camera", current_frame)
    
    # cv2.waitKey(1)

  def task_timeout_callback(self,msg):
    self.task_name = msg.name
    self.timeout = msg.timed_out
        
if __name__ == '__main__':
    detector = yolo_detector('/root/vrx_soln_ws/src/VRX_iitm/perception_soln/perception/model/obj.names', 
                          '/root/vrx_soln_ws/src/VRX_iitm/perception_soln/perception/cfg/yolov4-obj.cfg', 
                          '/root/vrx_soln_ws/src/VRX_iitm/perception_soln/perception/model/yolov4-obj_12500.weights')
