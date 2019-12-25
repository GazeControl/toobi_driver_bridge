#!/usr/bin/python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# Standard libraries. We need socket for socket communication and json for
# json parsing
import socket
import json
import pdb
import rospy

def mksock(peer):
    """ Create a socket pair for a peer description """
    iptype = socket.AF_INET
    if ':' in peer[0]:
        iptype = socket.AF_INET6
    sock = socket.socket(iptype, socket.SOCK_DGRAM)
    rospy.loginfo('Created new socket. socket fd no:{}'.format(sock.fileno()))
    return sock
