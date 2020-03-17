#!/usr/bin/env python
# Python libs
import socket, sys, json
from thread import *

# Ros libraries
import roslib, rospy, rospkg, rospy
import std_msgs.msg
import threading
# Ros Messages
from odas_msgs.msg import DoA, DoAs

import time

HOST = 'localhost'      # Symbolic name, meaning all available interfaces
PORTS = [9000, 9002]    # Arbitrary non-privileged port
PRINT = True
E_MOY_NUM = 25
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
        self.tracked_pub = rospy.Publisher("/DoA", DoAs, queue_size=1)
        # important variables
        self.list_socket = []
        self.nbr_socket = 2
        self.pot_dict = {}
        self.track_dict = {}
        self.publish_index = 1


        self.lastTimestamp = 0
        self.list_E = [0]*NUM_SOURCE

        self.raise_incomplete_Tracker = False
        self.raise_incomplete_Pot = False
        self.remaining_Tracker = []
        self.remaining_Pot = []

        for i in range(NUM_SOURCE):
            self.list_E[i] = [0.0000]*E_MOY_NUM
        self.reset_list_E = [0]*NUM_SOURCE
        self.last_E = [0.000]*NUM_SOURCE

        for i in range(self.nbr_socket):
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            s.bind((HOST, PORTS[i]))
            s.listen(5)
            self.list_socket.append(s)
            if PRINT : print "[*] Server listening on %s %d" %(HOST, PORTS[i])

    # def msgCompleter(self):
    # # Function that verifies if the json message is complete or incomplete

    def odas_publisher(self):
    # Function that verifies if a message is ready to be published and publishes it.
        print "in odas pub"
        while not rospy.is_shutdown():
            # print self.publish_index
            # print self.pot_dict
            timestamp = str(self.publish_index)
            if timestamp in self.pot_dict and timestamp in self.track_dict:
                #print 'okayy'
                # print self.pot_dict
                # print self.publish_index
                msg_ = DoAs()
                track_full = json.loads(self.track_dict[timestamp])
                pot_full = json.loads(self.pot_dict[timestamp])
                
                for k in range(len(track_full)):
                    # When every timestamps are good, we load the information from the msg into a publishable variable
                    source = DoA()
                    source.x = track_full[k]["x"]
                    source.y = track_full[k]["y"]
                    source.z = track_full[k]["z"]
                    source.activity = track_full[k]["activity"]
                    
                    # For every source in the pot message, in order of most energy to least
                    for l in range(len(pot_full)):
                        pot_x = pot_full[l]["x"]
                        pot_y = pot_full[l]["y"]
                        pot_z = pot_full[l]["z"]
                        pot_E = pot_full[l]["E"]

                        # If the source has the same x,y,z value as the one currently being observed, 
                        # we add its E value to the list of energy and erase the last value
                        if (source.x - TOL < pot_x < source.x + TOL) and (source.y - TOL < pot_y < source.y +TOL) and (source.z - TOL < pot_z < source.z + TOL):
                            self.list_E[k][1:] = self.list_E[k][:-1]
                            self.list_E[k][0] = pot_E

                    # The energy published is the average of the last E_MOY_NUM value of E
                    source.E = sum(self.list_E[k])/len(self.list_E[k])
                    
                    # If the source is not existant anymore (no change in E), reset the list
                    if source.E == self.last_E[k]:
                        self.reset_list_E[k] = self.reset_list_E[k] + 1
                        if self.reset_list_E[k] > 200:
                            source.E = 0.00
                            self.list_E[k] = [0.000]*E_MOY_NUM
                            self.reset_list_E[k] = 0
                    self.last_E[k] = source.E
                    
                    msg_.sources.append(source)

                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                msg_.header = h
                self.tracked_pub.publish(msg_)

                self.pot_dict.pop(str(self.publish_index))
                self.track_dict.pop(str(self.publish_index))
                self.publish_index = self.publish_index + 1
                


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
                    msg_ = Msg("tracker",message)

                    self.track_dict.update({msg_.timestamp:msg_.src})

                if len(test[-1]) > 0:
                    #print "Oh oh"
                    self.raise_incomplete_Tracker = True
                    self.remaining_Tracker.append(test[-1])
                    #print 'what is remaining: ', self.remaining_Tracker
            except ValueError:
                if PRINT : print "odas server not working socket Tracker(Take a look at soundusb card id)"
            #msg_.printMsg()
            
        conn.close()

    def socketPotential(self, conn):
    # Function receiving information from the odas potential socket and making sure it gets into a 
    # Msg class without being corrupted by the reading of the socket.
        while not rospy.is_shutdown():
            data = conn.recv(8192)
            test = data.split(']\n}\n')
            # print data
            # print "my test is  : ", test
            try:
                if self.raise_incomplete_Pot:
                    self.raise_incomplete_Pot = False
                    test[0] = self.remaining_Pot[0] + test[0]
                    self.remaining_Pot.pop(0)
                for message in test[0:-1:1]: 
                    message = message + ']\n}\n'
                    #print 'my message is : ', message, '\n\n'
                    msg_ = Msg("pot",message)
                    
                    self.pot_dict.update({msg_.timestamp:msg_.src})
                    
                if len(test[-1]) > 0:
                    self.raise_incomplete_Pot = True
                    self.remaining_Pot.append(test[-1])
            except ValueError:
                if PRINT : 
                    print "odas server not working socket Pot (Take a look at soundusb card id)"
            #self.MsgMatching()
            #msg_.printMsg()
        conn.close()    

    

    def spin(self):
        while not rospy.is_shutdown():
            connTrack, addrTrack = self.list_socket[0].accept()
            connPot, addrPot = self.list_socket[1].accept()
            if PRINT : print '[*] Connected with ' + addrTrack[0] + ':' + str(addrTrack[1])
            if PRINT : print '[*] Connected with ' + addrPot[0] + ':' + str(addrPot[1])
            start_new_thread(self.socketTracker ,(connTrack,))
            start_new_thread(self.socketPotential ,(connPot,))
            self.odas_publisher()
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
