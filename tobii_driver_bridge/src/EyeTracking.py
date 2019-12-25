#!/usr/bin/python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# Standard libraries. We need socket for socket communication and json for
# json parsing
import socket
import json
import pdb
import numpy as np

from utils import mksock
from KeepAlive import KeepAlive

import rospy
import roslib
from geometry_msgs.msg import PointStamped

# GStreamer integration. We need glib for main loop handling and gst for
# gstreamer video.
try:
    import glib
    import gst
except ImportError:
    print "Please make sure you have glib and gst python integration installed"
    raise

# calibration matrix for scene camera
K_matrix = np.array([[1133.0232, 0.00000000, 975.242159],
                     [0.00000000, 1133.23878, 542.78567],
                     [0.00000000, 0.0000000, 1.000000]])

HEIGHT = rospy.get_param("~height", 1080)
WIDTH = rospy.get_param("~width", 1920)

class EyeTracking():
    """ Read eyetracking data from RU """
    def __init__(self, publisher = None):
        self._ioc_sig = 0
        self._buffersync = None
        self.publisher = publisher
        self.gaze_stamped = PointStamped()

        self._gaze_in_pixel = np.ones(3)

    def start(self, et_sock, peer, buffersync):
        self._sock = et_sock
        self._keepalive = KeepAlive(self._sock, peer, "data")
        ioc = glib.IOChannel(self._sock.fileno())
        self._ioc_sig = ioc.add_watch(glib.IO_IN, self._data)
        self._buffersync = buffersync
        rospy.loginfo("Succeeded launching data stream pipeline")

    def _data(self, ioc, cond):
        """ Read next line of data """
        objs = json.loads(ioc.readline())
        self._buffersync.add_et(objs)
        self._publish_gaze_msg([objs])
        return True
   
    def stop(self):
        """ Stop the live data """
        self._keepalive.stop()
        self._keepalive = None
        self._sock.close()
        self._sock = None
        glib.source_remove(self._ioc_sig)

    def _publish_gaze_msg(self, objs):
        """ convert data into msg"""
        gaze_points_2d = filter(lambda x: 'gp' in x, objs)

        if len(gaze_points_2d) != 0:
            recent2d = gaze_points_2d[-1]
            # update the 2D gaze coordinate if recent point is valid
            if "gp" in recent2d :
                # transform from pixel frame into camera frame
                self._gaze_in_pixel[0] = recent2d["gp"][0]#*WIDTH
                self._gaze_in_pixel[1] = recent2d["gp"][1]#*HEIGHT
                # publish msgs
                current_time = rospy.Time.from_sec(recent2d["ts"]/1e6)
                self.gaze_stamped.header.stamp = current_time
                self.gaze_stamped.header.frame_id = "tobii_glasses"
                self.gaze_stamped.point.x = self._gaze_in_pixel[0]#gaze_in_meter[0]
                self.gaze_stamped.point.y = self._gaze_in_pixel[1]#gaze_in_meter[1]
                self.gaze_stamped.point.z = 1.#1.

                self.publisher.publish(self.gaze_stamped)
