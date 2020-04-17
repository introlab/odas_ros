#!/usr/bin/env python
# Python libs
import socket, sys, json
from thread import *
import rospy
import numpy as np

# Ros libraries
import roslib, rospy, rospkg, rospy
import std_msgs.msg
import threading
from odas_msgs.msg import UniqueSSS, Sss

import time

HOST = 'localhost'      # Symbolic name, meaning all available interfaces
PORTS = [10000]    # Arbitrary non-privileged port
PRINT = True
NUM_SOURCE = 4
TOL = 0.05      

class Odas:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        rospy.init_node('odas_driver', anonymous=True, disable_signals=True)
        self.sss_pub = rospy.Publisher("/SSS", Sss, queue_size=1)
        # important variables
        self.list_socket = []
        self.nbr_socket = 1


        self.raise_incomplete_Tracker = False
        self.remaining_Tracker = []

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        s.bind((HOST, PORTS[0]))
        s.listen(5)
        self.list_socket.append(s)
        if PRINT : print "[*] Server listening on %s %d" %(HOST, PORTS[0])


    def socketSSS(self, conn):
    # Function receiving information from the odas sss
        while not rospy.is_shutdown():
            start_time = time.time()
            data = conn.recv(8192) # recive in string      
            #print 'my datas are : ', data.encode('hex')

            nbr_channel = 4 
            dt = np.dtype('<i4')
            try:
                msg_ = Sss()
                source = UniqueSSS()
                data_32 = np.frombuffer(data, dtype=dt)
                data_block = data_32.reshape((len(data_32)/nbr_channel,nbr_channel))
                #print 'my datas are : ', data_block

                for i in range(nbr_channel):
                    source.sss_channels[i].raw_data = data_block[:,i]

                msg_.sources.append(source)
                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                #print msg.timestamp
                msg_.header = h
                self.sss_pub.publish(msg_)

            except ValueError:
                if PRINT : print "odas server not working socket SSS(Take a look at soundusb card id)"
            end_time = time.time()
            #print 'it took ' + str(end_time-start_time) + ' to run the loop'
        conn.close()
    

    def spin(self):
        while not rospy.is_shutdown():
            connSSS, addrSSS = self.list_socket[0].accept()
            if PRINT : print '[*] Connected with ' + addrSSS[0] + ':' + str(addrSSS[1])
            self.socketSSS(connSSS)
        s.close()

def main(args):
    '''Initializes and cleanup ros node'''
    odas_ = Odas()
    try:
        odas_.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS ODAS driver"

if __name__ == '__main__':
    if PRINT : print "Hello World!\n"
    main(sys.argv)
    if PRINT : print "GoodBye World!\n"
