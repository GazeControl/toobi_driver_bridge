#!/usr/bin/python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# Standard libraries. We need socket for socket communication and json for
# json parsing
import json

from KeepAlive import KeepAlive

import rospy
from geometry_msgs.msg import PointStamped

# GStreamer integration. We need glib for main loop handling and gst for
# gstreamer video.
try:
    import glib
except ImportError:
    print "Please make sure you have glib and gst python integration installed"
    raise

# calibration matrix for scene camera

class EyeTracking():
    """ Read eyetracking data from RU """
    def __init__(self):
        self._ioc_sig = 0

    def start(self, et_sock, peer):
        self._sock = et_sock
        self._keepalive = KeepAlive(self._sock, peer, "data")
        ioc = glib.IOChannel(self._sock.fileno())
        self._ioc_sig = ioc.add_watch(glib.IO_IN, self._data)
        rospy.loginfo("Succeeded launching data stream pipeline")

    def _data(self, ioc, cond):
        """ Read next line of data """
        objs = json.loads(ioc.readline())
        return True
   
    def stop(self):
        """ Stop the live data """
        self._keepalive.stop()
        self._keepalive = None
        self._sock.close()
        self._sock = None
        glib.source_remove(self._ioc_sig)


