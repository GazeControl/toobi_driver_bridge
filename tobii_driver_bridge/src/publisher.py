#!/usr/bin/env python
import time
import urllib2
import json
import signal

# GStreamer integration. We need glib for main loop handling and gst for
# gstreamer video.
try:
    import glib
    import gst
except ImportError:
    print "Please make sure you have glib and gst python integration installed"
    raise

from EyeTracking import EyeTracking
from VideoNoText import VideoNoText
from utils import mksock

import rospy


class ROSTobiiGlasses(object):
    def __init__(self):
        rospy.init_node("tobii_driver_bridge")
                
        height = rospy.get_param("~height", 1080)
        width = rospy.get_param("~width", 1920)
        self.ipaddr = rospy.get_param("~ip")
                
        self.video = VideoNoText(height, width)
        self.eye_tracking = EyeTracking()


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

        # create IO socket
        et_socket = mksock(peer)
        video_socket = mksock(peer)

        # check for calibration status
        calibrate = rospy.get_param("~calibrate", True)
        if calibrate:
            self.calibrate()
        else:
            rospy.logwarn("Skipping Calibration")

        # start gstreamer
        glib.threads_init()
        ml = glib.MainLoop()
        self.video.start(video_socket, peer)
        self.eye_tracking.start(et_socket, peer)

        self.set_glasses_param()
        # start grabbing video from the gstreamer stream
        self.video._start_video()

        ml.run() # run the inner loop



        
if __name__=="__main__":
    node = ROSTobiiGlasses()
    node.spin()
