#! /usr/bin/env python

#example subscriber that reconstructs video with 2d gaze points from the publisher
#publishes combined image on topic "gaze/recombined"

import rospy
import numpy as np
import datetime
from sensor_msgs.msg import Image
from tobii_msgs.msg import Gaze, GazeCoordinate2D
import cv2
from cv_bridge import CvBridge, CvBridgeError
from decimal import Decimal

HEIGHT = rospy.get_param("~height", 1080)
WIDTH = rospy.get_param("~width", 1920)
combined_image = np.zeros((HEIGHT,WIDTH,3), np.uint8)

def gaze_callback(Gaze):
    #print "Received a gaze message. Time: " + str(datetime.datetime.now().time().isoformat())

    global combined_image 

    #convert ROS image portion of Gaze message to an OpenCV image
    combined_image = CvBridge().imgmsg_to_cv2(Gaze.image, desired_encoding="passthrough")

    #draw point on image using GazeCoordinate2D portion of Gaze message
    cv2.circle(combined_image, (int(Gaze.coordinate2d.x * WIDTH),int(Gaze.coordinate2d.y * HEIGHT)), 12, (0,255,0), -1)

if __name__ == "__main__":
    
    combined_vid_pub = rospy.Publisher("gaze/recombined", Image, queue_size=1)
    rospy.init_node("tobii_combiner", anonymous = True)
    rate = rospy.Rate(25)
    rospy.Subscriber("gaze", Gaze, gaze_callback)

    while not rospy.is_shutdown():
        try:
            combined_image_msg = CvBridge().cv2_to_imgmsg(combined_image, encoding="bgr8")
            combined_image_msg.header.stamp = rospy.get_rostime()
	    combined_vid_pub.publish(combined_image_msg)
        except CvBridgeError as e:
            print (e)
        rate.sleep()
