#!/usr/bin/env python
# Python libs
import socket, sys, json
from thread import *
import rospy

# Ros libraries
import roslib, rospy, rospkg, rospy
import std_msgs.msg
import threading
# Ros Messages (todo -> creer odas_msgs SSS)
from odas_msgs.msg import UniqueSSS, Sss

import time

HOST = 'localhost'      # Symbolic name, meaning all available interfaces
PORTS = [10000]    # Arbitrary non-privileged port
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
        self.sss_pub = rospy.Publisher("/SSS", Sss, queue_size=1)
        # important variables
        self.list_socket = []
        self.nbr_socket = 1
        self.time_stamp_HARD = 0


        self.raise_incomplete_Tracker = False
        self.remaining_Tracker = []

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        s.bind((HOST, PORTS[0]))
        s.listen(5)
        self.list_socket.append(s)
        if PRINT : print "[*] Server listening on %s %d" %(HOST, PORTS[0])

    # def msgCompleter(self):
    # # Function that verifies if the json message is complete or incomplete
                


    def socketSSS(self, conn):
    # Function receiving information from the odas tracker socket and making sure it gets into a 
    # Msg class without being corrupted by the reading of the socket.
        freq = 20.0
        rate = rospy.Rate(freq)
        #data = bytearray(8196)
        while not rospy.is_shutdown():
            start_time = time.time()
            data = conn.recv(8192) # recive in string, not used anymore
            #conn.recv_into(data) #-> TODO brise car UniqueSSS est array now 
            #for index in range(len(data)):
            #    print 'my data' + str(index) + 'is: ', data[index]#.encode('hex')       
            #print 'my datas are : ', data.encode('hex')
            n = 32/8 # odas raw size / python str size
            channe1 = []
            channe2 = []
            channe3 = []
            channe4 = []
            data_by_channel = [channe1, channe2, channe3, channe4] 
            index_channel = 0
            try:
                msg_ = Sss()
                source = UniqueSSS()

                for i in range(0, len(data), n):
                    data_now = data[i:i+n]
                    data_by_channel[index_channel].append(data_now.encode('hex'))

                    if(index_channel == 0):
                        source.sss_channel_1 = data_now

                    elif(index_channel == 1):
                        source.sss_channel_2 = data_now

                    elif(index_channel == 2):
                        source.sss_channel_3 = data_now

                    elif(index_channel >= 3):
                        source.sss_channel_4 = data_now
                        msg_.sources.append(source)
                        index_channel = -1

                    index_channel = index_channel + 1                        

                print 'my channel_1 ('+ str(len(data_by_channel[0])) +') is : ', data_by_channel[0]
                print 'my channel_2 ('+ str(len(data_by_channel[1])) +') is : ', data_by_channel[1]
                print 'my channel_3 ('+ str(len(data_by_channel[2])) +') is : ', data_by_channel[2]
                print 'my channel_4 ('+ str(len(data_by_channel[3])) +') is : ', data_by_channel[3]
                
                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                #print msg.timestamp
                h.frame_id = str(self.time_stamp_HARD) #msg.timestamp 
                self.time_stamp_HARD = self.time_stamp_HARD + 1
                msg_.header = h
                self.sss_pub.publish(msg_)

            except ValueError:
                if PRINT : print "odas server not working socket Tracker(Take a look at soundusb card id)"
            #msg_.printMsg()
            end_time = time.time()
            print 'it took ' + str(end_time-start_time) + ' to run the loop'
            if((end_time-start_time) <= (1.0/freq)):
                print 'succes'
            else:
                print 'fail'
            rate.sleep()
        conn.close()

    

    def spin(self):
        while not rospy.is_shutdown():
            connTrack, addrTrack = self.list_socket[0].accept()
            if PRINT : print '[*] Connected with ' + addrTrack[0] + ':' + str(addrTrack[1])
            self.socketSSS(connTrack)
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
