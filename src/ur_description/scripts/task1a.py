#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          CL#1530
# Author List:		Pratham, Adithya, Swastik, Rayner
# Filename:		    task1a.py
# Functions:        [calculate_rectangle_area, detect_aruco, depthimagecb, colorimagecb, process_image, main]
# Nodes:		    
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /camera/color/image_raw ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros 
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import time


##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ###########.

    area = None
    width = None

    area = cv2.contourArea(coordinates)
    width = math.sqrt(area)                                     # Since aruco is basically a square
    return area, width


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # Initializing the main variables
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
    corners = []
    rejected = []
 
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray_image, arucoDict, parameters=arucoParams)
    if ids is not None:
        for i in range(len(ids)):
            # Calculate the marker center
            c = corners[i][0]
            area, width = calculate_rectangle_area(c)
            if(area < aruco_area_threshold):                        # Removing the markers that do not meet the threshold requirement
                ids = np.delete(ids,i)
                continue
            marker_center = np.mean(c, axis=0)
            center_aruco_list.append(marker_center)

            #Pose estimation
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_m, cam_mat, dist_mat)

            distance_from_rgb_list.append(tvec)
            angle_aruco_list.append(rvec)
            # print(f"For id {ids[i]} :  ",rvec)
            width_aruco_list.append(width)

    return((center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids , corners))


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        self.test = 10
        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
         

    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)

    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data)
            self.center_aruco_list, self.distance_from_rgb_list, self.angle_aruco_list, self.width_aruco_list, self.ids , self.corners = detect_aruco(self.cv_image)
            angles = self.angle_aruco_list
            self.process_image()
        except CvBridgeError as e:
            print(e)

    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
        dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])  

        # Code to add frames to the image
       
        if self.ids is not None:
            ids = self.ids
            for i in range(len(ids)):
                cv2.drawFrameAxes(self.cv_image, cam_mat, dist_mat, self.angle_aruco_list[i], self.distance_from_rgb_list[i], 0.15)
                cv2.aruco.drawDetectedMarkers(self.cv_image, self.corners)
                cv2.circle(self.cv_image, (int(self.center_aruco_list[i][0]), int(self.center_aruco_list[i][1])), 5, (0, 0, 255), -1)
    
        # Code to Initialize and publish the transorms

        if self.ids is not None:
            ids = self.ids
            for i in range(len(ids)):
                d = np.linalg.norm(self.distance_from_rgb_list[i][0][0])
                cX,cY= (self.center_aruco_list[i])
                x = d * (sizeCamX - cX - centerCamX) / focalX
                y = d * (sizeCamY - cY - centerCamY) / focalY
                z = d      
                transform=TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id='camera_link'
                transform.child_frame_id=f'cam_{ids[i]}'

                # writing the transforms
                transform.transform.translation.x = z
                transform.transform.translation.y = x
                transform.transform.translation.z = y
   
                angle = self.angle_aruco_list[i][0][0]

                # Cordinates with respect to camera cordinate system
                from_opencv = R.from_rotvec(angle).as_euler('xyz', degrees = True)
                to_ros = np.array([-from_opencv[0]-90,-from_opencv[1],from_opencv[2]])

                # Transform with respect to world cordinate system
                p = R.from_euler('yzx',to_ros, degrees = True)

                q = R.from_euler('z',90,degrees = True)

                # Combining the transforms
                r = p*q

                quat = r.as_quat()
                transform.transform.rotation.x = quat[0]
                transform.transform.rotation.y = quat[1]
                transform.transform.rotation.z = quat[2]
                transform.transform.rotation.w = quat[3]
                self.br.sendTransform(transform)

        newTransform = TransformStamped()
        newTransform.header.stamp = self.get_clock().now().to_msg()
        newTransform.header.frame_id='base_link'
        newTransform.child_frame_id=f'obj_{ids[i]}'

        if ids is not None:
            for i in range(len(ids)):
                # Looking up the relative transform
                try:
                    rel = self.tf_buffer.lookup_transform('base_link', f'cam_{ids[i]}', rclpy.time.Time())
                    time.sleep(0.1)
                    newTransform = TransformStamped()
                    newTransform.header.stamp = self.get_clock().now().to_msg()
                    newTransform.header.frame_id='base_link'
                    newTransform.child_frame_id=f'obj_{ids[i]}'
                    newTransform.transform.translation.x = rel.transform.translation.x
                    newTransform.transform.translation.y = rel.transform.translation.y
                    newTransform.transform.translation.z = rel.transform.translation.z
                    newTransform.transform.rotation.x = rel.transform.rotation.x
                    newTransform.transform.rotation.y = rel.transform.rotation.y
                    newTransform.transform.rotation.z = rel.transform.rotation.z
                    newTransform.transform.rotation.w = rel.transform.rotation.w
                    self.br.sendTransform(newTransform)
                    print("Transform Published")
                except tf2_ros.TransformException as ex:
                    self.get_logger().info("No transform found")

        ## Optional Code to display the image with the aruco marks Marked On it
        # Display the image with markers and centers
        # cv2.imshow('Image with ArUco Markers', self.cv_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
 


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')       # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'
    try:
        rate = aruco_tf_class.create_rate(1)
        rclpy.spin(aruco_tf_class)                                  # spining on the object to make it alive in ROS 2 DDS
        
        while rclpy.ok:
            rate.sleep()
    except KeyboardInterrupt:
        pass

    aruco_tf_class.destroy_node()
    rclpy.shutdown()
                                        
                                     

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()