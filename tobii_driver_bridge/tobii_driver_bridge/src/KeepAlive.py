#!/usr/bin/python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# Standard libraries. We need socket for socket communication and json for
# json parsing
import socket
import json
import pdb

from utils import mksock

# GStreamer integration. We need glib for main loop handling and gst for
# gstreamer video.
try:
    import glib
    import gst
except ImportError:
    print "Please make sure you have glib and gst python integration installed"
    raise

class KeepAlive():
    """ 
    Sends keep-alive signals to a peer via a socket 
    """
    _sig = 0
    def __init__(self, sock, peer, streamtype, key='anything',timeout=1):
    # Sends keep-alive signals to a peer via a socket
        jsonobj = json.dumps({
            'op' : 'start',
            'type' : ".".join(["live", streamtype, "unicast"]),
            'key' : key})
        if isinstance(peer, tuple):
            sock.sendto(jsonobj, peer)
        else:
            sock.send_to(peer, jsonobj)
        self._sig = glib.timeout_add_seconds(timeout, self._timeout, sock, peer, jsonobj)

    def _timeout(self, sock, peer, jsonobj):
        if isinstance(peer, tuple):
            sock.sendto(jsonobj, peer)
        else:
            sock.send_to(peer, jsonobj)
        return True

    def stop(self):
        glib.source_remove(self._sig)
