 #!/usr/bin/python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# Standard libraries. We need socket for socket communication and json for
# json parsing
import rospy
import cv2
import numpy as np

from KeepAlive import KeepAlive

# GStreamer integration. We need glib for main loop handling and gst for
# gstreamer video.
try:
    import gst
except ImportError:
    print "Please make sure you have glib and gst python integration installed"
    raise

# video class that removed the text overlay on the video stream
class VideoNoText(object):
    ''' Setup Video class to decode and publish the images instead
    ' Video subclass that removes the text overlay and prevents gstreamer from
    ' starting upon object creation.
    '''
    
    _PIPEDEF=[
        "udpsrc name=src blocksize=1316 closefd=false buffer-size=5600", # UDP video data
        "tsparse",                      # parse the incoming stream to MPegTS
        "tsdemux emit-stats=True",      # get pts statistics from the MPegTS stream
        "queue",                        # build queue for the decoder 
        "ffdec_h264 max-threads=1",     # decode the incoming stream to frames
        "identity name=decoded",        # used to grab video frame to be displayed
        "fakesink sync=false"           # dump video frames from memory after all relevant processing is done
    ]
    
    def __init__(self, height, width, channels = 3):
        self._height = height
        self._width = width
        self._channels = channels
        self._size = width * height

    #Function to convert from h264 to RGB for ROS purposes
    def YUV_stream2RGB_frame(self, data):
        stream = np.fromstring(data, np.uint8) #Convert data from string to numpy array

        #Y bytes will start from 0 and end in size-1
        y = stream[0:self._size].reshape(self._height, self._width)
        #U bytes will start from size and end at size+size/4 as its size = framesize/4
        u = stream[self._size:(self._size+(self._size/4))].reshape((self._height/2), (self._width/2))
        #Up-sample the u channel to be the same size as the y channel and the frame 
        u_upsize = cv2.pyrUp(u)
        #Do the same for the v channel
        v = stream[(self._size+(self._size/4)):].reshape((self._height/2),(self._width/2))
        v_upsize = cv2.pyrUp(v)
        #Create the 3-channel frame using cv2.merge function
        yuv = cv2.merge((y, u_upsize, v_upsize))
        #Convert to RGB format
        rgb = cv2.cvtColor(yuv, cv2.COLOR_YCR_CB2RGB)
        return rgb

    def start(self, video_sock, peer):
        rospy.loginfo("starting video stream")
        """ Prepare to start grabbing video """
        self._sock = video_sock

        #Create pipeline
        self._pipeline=gst.parse_launch(" ! ".join(self._PIPEDEF))
        rospy.loginfo("gstreamer pipeline has been launched")

        #Add watch to pipeline to get tsdemux messages
        bus = self._pipeline.get_bus()
        bus.add_watch(self._bus)

        # Set socket to get data from out socket
        src = self._pipeline.get_by_name("src")
        src.set_property("sockfd", self._sock.fileno())
        src.set_property ("buffer-size", 100000)
        srcpad = src.get_static_pad("udpsrc")

        #Catch decoded frames
        decoded = self._pipeline.get_by_name("decoded")
        decoded.connect("handoff", self._decoded_buffer)

        self._keepalive = KeepAlive(self._sock, peer, "video", "anything")

    def _start_video(self):
        """Start grapping video from pipeline"""
        self._pipeline.set_state(gst.STATE_PLAYING)
        rospy.loginfo("video stream started.")

    def _decoded_buffer(self, ident, buf):
        """Decode the video into a portable format"""
        #nowts = self._buffersync.flush_pts(buf.offset, buf.timestamp/1000)
        image = self.YUV_stream2RGB_frame(buf)
#        self._image_ts = nowts
#        self.image_queue.append((self._image, self._image_ts))
#        if len(self.image_queue) > 2 and not self.video_ready:
#            self.video_ready = True

    def _bus(self, bus, msg):
        """ Buss message handler 
        " We only collect pts-offset pairs
        """
        if msg.type == gst.MESSAGE_ELEMENT:
            st = msg.structure
            # If we have a tsdemux message with pts field then lets store it
            # for the render pipeline. Will be picked up by the handoff
            if st.has_name("tsdemux") and st.has_field("pts"):
                #self._buffersync.add_pts_offset(st['offset'], st['pts'])
        return True

    def stop(self):
        self._pipeline.set_state(gst.STATE_NULL)
        self._pipeline = None
        self._sock.close()
        self._sock = None
        self._keepalive.stop()
        self._keepalive = None

    @property
    def image(self):
        return self._image

    @property
    def image_ts(self):
        return self._image_ts
