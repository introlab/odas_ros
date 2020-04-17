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
# Ros Messages
from odas_msgs.msg import UniqueSST, Sst
from odas_msgs.msg import UniqueSSS, Sss

import time

HOST = 'localhost'      # Symbolic name, meaning all available interfaces
PORTS = [9000, 10000]    # Arbitrary non-privileged port
PRINT = True
NUM_SOURCE = 4
TOL = 0.05

class Msg:
    #Class containning the information of the messages read on the sockets json sent from odas
    def __init__(self, name, data):
        self.name = name
        self.data = json.loads(data)
        self.timestamp = json.dumps(self.data["timeStamp"])
        self.src = json.dumps(self.data["src"])
    
    def printMsg(self):
        if PRINT : 
            print "RX %s = \nTimeStamp = %s,\nsrc = %s" %(self.name, self.timestamp, json.dumps(self.src))
        

class Odas:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        rospy.init_node('odas_driver', anonymous=True, disable_signals=True)
        self.tracked_pub = rospy.Publisher("/SST", Sst, queue_size=1)
        self.sss_pub = rospy.Publisher("/SSS", Sss, queue_size=1)
        # important variables
        self.list_socket = []
        self.nbr_socket = 2
        self.track_dict = {}


        self.raise_incomplete_Tracker = False
        self.remaining_Tracker = []
        
        for i in range(self.nbr_socket):
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            s.bind((HOST, PORTS[i]))
            s.listen(5)
            self.list_socket.append(s)
            if PRINT : print "[*] Server listening on %s %d" %(HOST, PORTS[i])

    # def msgCompleter(self):
    # # Function that verifies if the json message is complete or incomplete
                


    def socketTracker(self, conn):
    # Function receiving information from the odas tracker socket and making sure it gets into a 
    # Msg class without being corrupted by the reading of the socket.
        while not rospy.is_shutdown():
            data = conn.recv(8192)
            test = data.split(']\n}\n')
            #print len(test[-1]), len(test[0])
            #print 'my datas are : ', data
            try:
                if self.raise_incomplete_Tracker:
                    #print "oh oh i will solve dis"
                    self.raise_incomplete_Tracker = False
                    test[0] = self.remaining_Tracker[0] + test[0]
                    self.remaining_Tracker.pop(0)
                    #print 'the modified test is: ', test[0]
                    #print 'what is remaining: ', self.remaining_Tracker
                for message in test[0:-1:1]:
                    message = message + ' ]\n}\n'
                    #print 'my messages are : ', message, '\n\n'
                    msg = Msg("tracker",message)
                    self.track_dict.update({msg.timestamp:msg.src})
                    msg_ = Sst()
                    track_full = json.loads(self.track_dict[msg.timestamp])                    
                    for k in range(len(track_full)):
                        # When every timestamps are good, we load the information from the msg into a publishable variable
                        source = UniqueSST()
                        source.x = track_full[k]["x"]
                        source.y = track_full[k]["y"]
                        source.z = track_full[k]["z"]
                        source.activity = track_full[k]["activity"]
                        
                        # For every source in the pot message, in order of most energy to least
                        
                        msg_.sources.append(source)

                    h = std_msgs.msg.Header()
                    h.stamp = rospy.Time.now()
                    #print msg.timestamp
                    h.seq = msg.timestamp 
                    msg_.header = h
                    self.tracked_pub.publish(msg_)

                    self.track_dict.pop(msg.timestamp)

                    

                if len(test[-1]) > 0:
                    #print "Oh oh"
                    self.raise_incomplete_Tracker = True
                    self.remaining_Tracker.append(test[-1])
                    #print 'what is remaining: ', self.remaining_Tracker
            except ValueError:
                if PRINT : print "odas server not working socket Tracker(Take a look at soundusb card id)"
            #msg_.printMsg()
            
        conn.close()

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
            connTrack, addrTrack = self.list_socket[0].accept()
            connSSS, addrSSS = self.list_socket[1].accept()
            if PRINT : print '[*] Connected with ' + addrTrack[0] + ':' + str(addrTrack[1])
            if PRINT : print '[*] Connected with ' + addrSSS[0] + ':' + str(addrSSS[1])
            start_new_thread(self.socketTracker ,(connTrack,))
            start_new_thread(self.socketSSS ,(connSSS,))
            self.socketTracker(connTrack)
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
