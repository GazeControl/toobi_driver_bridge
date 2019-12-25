#!/usr/bin/env python
from collections import namedtuple
import time
import urllib2
import json
import cv2
import numpy as np
import signal
import Queue
import os
import math

# GStreamer integration. We need glib for main loop handling and gst for
# gstreamer video.
try:
    import glib
    import gst
except ImportError:
    print "Please make sure you have glib and gst python integration installed"
    raise

from EyeTracking import EyeTracking
from BufferSync import BufferSync
from VideoNoText import VideoNoText
from utils import mksock

import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from tobii_msgs.msg import Gaze, GazeCoordinate2D, GazeCoordinate3D
from geometry_msgs.msg import PointStamped


class Coordinate2D:
    def __init__ (self, x, y, t):
        self.x, self.y = x, y
        self.t = t

class Coordinate3D:
    def __init__ (self, x, y, z, t):
        self.x, self.y, self.z = x, y, z
        self.t = t

class ROSTobiiGlasses(object):
    def __init__(self):
        rospy.init_node("tobii_bridge")
        
        self.gaze_pub = rospy.Publisher("gaze", Gaze, queue_size=1)
        self.video_only_pub = rospy.Publisher("gaze/image", Image, queue_size=1)
        self.gaze_2d_coord_pub = rospy.Publisher("gaze/coordinate2d", 
                                                 GazeCoordinate2D, 
                                                 queue_size=1)
        self.gaze_3d_coord_pub = rospy.Publisher("gaze/coordinate3d", 
                                                 GazeCoordinate3D, 
                                                 queue_size=1)
        self.imu_pub = rospy.Publisher("gaze/imu", Imu, queue_size=1)
        self.gaze_3d_coord_stamped_pub = rospy.Publisher("gaze/coordinate3d_stamped",
                                                         PointStamped,
                                                         queue_size=1)
        
        height = rospy.get_param("~height", 1080)
        width = rospy.get_param("~width", 1920)
        #self.ipaddr = rospy.get_param("~ip", "192.168.71.50")
        self.ipaddr = "fe80::76fe:48ff:fe2d:583d%eno1"
        
        self.coordinate2d = Coordinate2D(0,0, rospy.get_rostime())
        self.coordinate3d = Coordinate3D(0,0,0, rospy.get_rostime())
        
        self.video = VideoNoText(height, width)
        self.eye_tracking = EyeTracking(self.gaze_3d_coord_stamped_pub)
        self.buffersync = BufferSync(self.updateGaze, self.imu_pub)        

        self._next_pts = None

    def post_request(self, api_action, data=None):
        if ":" in self.ipaddr:
            url = "http://[" + self.ipaddr + "]" + api_action
        else:
            url = "http://" + self.ipaddr + api_action
        req = urllib2.Request(url)
        req.add_header('Content-Type', 'application/json')
        data = json.dumps(data)
        response = urllib2.urlopen(req, data)
        data = response.read()
        json_data = json.loads(data)
        return json_data

    def wait_for_status(self, api_action, key, values):
        if ":" in self.ipaddr:
            url = "http://[" + self.ipaddr + "]" + api_action
        else:
            url = "http://" + self.ipaddr + api_action
        running = True
        while running:
            req = urllib2.Request(url)
            req.add_header('Content-Type', 'application/json')
            response = urllib2.urlopen(req, None)
            data = response.read()
            json_data = json.loads(data)
            if json_data[key] in values:
                running = False
            time.sleep(1)
        return json_data[key]

    def create_project(self):
        json_data = self.post_request('/api/projects')
        return json_data['pr_id']

    def create_participant(self, project_id):
        data = {'pa_project': project_id}
        json_data = self.post_request('/api/participants', data)
        return json_data['pa_id']

    def create_calibration(self, project_id, participant_id):
        data = {'ca_project': project_id,
                'ca_type': 'default',
                'ca_participant': participant_id}
        json_data = self.post_request('/api/calibrations', data)
        return json_data['ca_id']

    def start_calibration(self, calibration_id):
        self.post_request('/api/calibrations/' + calibration_id + '/start')
        
    def calibrate(self):
        # number of seconds till 
        timeout = rospy.get_param("~timeout", 30)

        project_id = self.create_project()
        participant_id = self.create_participant(project_id)
        start = time.time()

        # attempt to calibrate until timeout
        while (time.time() - start) < timeout:
            rospy.loginfo("Tobii glasses: Calibration Started")

            calibration_id = self.create_calibration(project_id, participant_id)
            
            self.start_calibration(calibration_id)

            api_action = '/api/calibrations/' + calibration_id + '/status'
            status = self.wait_for_status(api_action,
                                          'ca_state',
                                          ['failed', 'calibrated'])
            
            if status == "calibrated":
                rospy.loginfo("Tobii glasses: calibration complete")
                return True
            
        rospy.logerr("Tobii glasses: calibration failed")
        return False
        
    def set_glasses_param(self):
        sys_sc_preset =rospy.get_param("~sys_sc_preset", "Auto")
        sys_ec_preset = rospy.get_param("~sys_ec_preset", "Indoor")
        data = {'sys_sc_preset':sys_sc_preset,
                'sys_ec_preset':sys_ec_preset}
        self.post_request('/api/system/conf', data)

    def _signal_handler(signal, frame):
        self.video.stop()
        self.eye_tracking.stop()
        sys.exit(0)

    def spin(self):
        if rospy.get_param("~debug", True):
            gst.debug_set_active(True)
            gst.debug_set_default_threshold(2)

        # catch break signal for program stop
        signal.signal(signal.SIGINT, self._signal_handler)

        port = rospy.get_param("port", 49152)
        peer = (self.ipaddr, port)
        # peer = Peer(host = self.ipaddr, port = port)

        # create IO socket
        et_socket = mksock(peer)
        video_socket = mksock(peer)

        # check for calibration status
        calibrate = rospy.get_param("~calibrate", True)
        if calibrate:
            if self.calibrate():
                os.system("aplay `rospack find tobii_driver`/media/notification.wav")
        else:
            rospy.logwarn("Skipping Calibration")

        # start gstreamer
        glib.threads_init()
        ml = glib.MainLoop()
        self.video.start(video_socket, peer, self.buffersync)
        self.eye_tracking.start(et_socket, peer, self.buffersync)
        

        self.set_glasses_param()
        # start grabbing video from the gstreamer stream
        self.video._start_video()

        # run the timer at given sampling rate
        self.rate = rospy.get_param("~rate", 25.0)
        self._frame_duration = (int)(1e6/self.rate) # in micro sec
        # carefully change this parameter
        self.imu_rate = rospy.get_param("~imu_rate", 90.0)
        rospy.Timer(rospy.Duration(1.0/self.imu_rate), self.publishImu)
        rospy.Timer(rospy.Duration(1.0/self.rate), self.publishGaze)
        
        ml.run() # run the inner loop
        
    def updateGaze(self, objs):
        gaze_points_2d = filter(lambda x: 'gp' in x, objs)
        gaze_points_3d = filter(lambda y: 'gp3' in y, objs)
        
        if len(gaze_points_2d) != 0:
            recent2d = gaze_points_2d[-1]
            # update the 2D gaze coordinate if recent point is valid
            if "gp" in recent2d and recent2d["gp"][0] != 0 and recent2d["gp"][1] != 0:
                self.coordinate2d.x = recent2d["gp"][0]
                self.coordinate2d.y = recent2d["gp"][1]
                self.coordinate2d.t = rospy.Time.from_sec(recent2d["ts"]/1e6)

        if len(gaze_points_3d) != 0:
            recent3d = gaze_points_3d[-1]
            # update the 3D gaze coordinate if recent point is valid
            if "gp3" in recent3d and recent3d["gp3"][0] != 0 and recent3d["gp3"][1] != 0 and recent3d["gp3"][2] != 0:
                self.coordinate3d.x = recent3d["gp3"][0]
                self.coordinate3d.y = recent3d["gp3"][1]
		self.coordinate3d.z = recent3d["gp3"][2]
		self.coordinate3d.t = rospy.Time.from_sec(recent3d["ts"]/1e6)
            
    def publishGaze(self, event):
        if not self.video.video_ready:
            rospy.loginfo_throttle(2, "preparing for catching the first frame...")
            return 

        if self._next_pts is None:
            # this condition considers the first frame coming in
            self._next_pts = self.video.image_queue[0][1]

        if(self.video.image_queue == [] and self.video.video_ready):
            self._next_pts += self._frame_duration
            rospy.logwarn("video source queue empty! drop one image frame")
            return

        # add 0.001 sec tolorance for image frame selection
        tol = 1000
        images_past = filter(lambda x: x[1] <= self._next_pts+tol, self.video.image_queue)
        self.video.image_queue = filter(lambda x: x[1] > self._next_pts+tol, self.video.image_queue)
        # handle image packets drops
        if images_past != []:
            image_pair = images_past[-1]
        else:
            rospy.logerr("video queue error! drop one image frame. current time {}".format(self._next_pts))
            self._next_pts += self._frame_duration
            return

        try:
            ros_img_msg = CvBridge().cv2_to_imgmsg(image_pair[0],
                                                   encoding="bgr8")
            nowts = image_pair[1]/1e6
            ros_img_msg.header.stamp = rospy.Time.from_sec(nowts)
            self._next_pts += self._frame_duration

        except CvBridgeError:
            rospy.logerr("Could not convert image to cv2 spec!")
            return

        self.video_only_pub.publish(ros_img_msg)
        rospy.loginfo_throttle(1, "video queue length:{}".format(len(self.video.image_queue)))

        gaze_2d_coord_msg = GazeCoordinate2D()
        gaze_2d_coord_msg.header.stamp = self.coordinate2d.t
        gaze_2d_coord_msg.x = self.coordinate2d.x
        gaze_2d_coord_msg.y = self.coordinate2d.y
        self.gaze_2d_coord_pub.publish(gaze_2d_coord_msg)

        gaze_3d_coord_msg = GazeCoordinate3D()
        gaze_3d_coord_msg.header.stamp = self.coordinate3d.t
        gaze_3d_coord_msg.x = self.coordinate3d.x
        gaze_3d_coord_msg.y = self.coordinate3d.y
        gaze_3d_coord_msg.z = self.coordinate3d.z
        self.gaze_3d_coord_pub.publish(gaze_3d_coord_msg)
        
        gaze_msg = Gaze()
        gaze_msg.image = ros_img_msg
        gaze_msg.coordinate2d = gaze_2d_coord_msg
        gaze_msg.coordinate3d = gaze_3d_coord_msg
        self.gaze_pub.publish(gaze_msg)        

    def publishImu(self, event):
        self.buffersync.flush_imu_data(self.imu_rate, self.video.video_ready)
        
if __name__=="__main__":
    node = ROSTobiiGlasses()
    node.spin()
