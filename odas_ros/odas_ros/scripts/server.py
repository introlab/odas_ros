#!/usr/bin/env python
# Python libs
import socket, sys, json
from thread import *

# Ros libraries
import roslib, rospy, rospkg, rospy
import std_msgs.msg

# Ros Messages
from odas_msgs.msg import Tracked_source
from odas_msgs.msg import Tracked_sources

HOST = 'localhost'      # Symbolic name, meaning all available interfaces
PORTS = [9000, 9001, 9002]    # Arbitrary non-privileged port
PRINT = True
E_MOY_NUM = 25
NUM_SOURCE = 4
TOL = 0.05

class Msg:
    def __init__(self, name, data):
        self.name = name
        self.data = json.loads(data)
        self.timestamp = json.dumps(self.data["timeStamp"])
        self.src = json.dumps(self.data["src"])
    
    def printMsg(self):
        if PRINT : print "RX %s = \nTimeStamp = %s,\nsrc = %s" %(self.name, self.timestamp, json.dumps(self.src))
        

class odas:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.tracked_pub = rospy.Publisher("/tracked_sources", Tracked_sources, queue_size=1)
        self.list_socket = []
        self.nbr_socket = 3
        self.list_msg_tracker = []
        self.list_msg_speech = []
        self.list_msg_pot = []
        self.lastTimestamp = 0
        self.list_E = [0]*NUM_SOURCE
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

    def MsgMatching(self):
        for i in range(len(self.list_msg_tracker)):
            for j in range(len(self.list_msg_speech)):
                for p in range(len(self.list_msg_pot)):
                    if self.list_msg_tracker[i].timestamp == self.list_msg_speech[j].timestamp == self.list_msg_pot[p].timestamp:
                        self.lastTimestamp = self.list_msg_speech[j].timestamp
                        msg_ = Tracked_sources()
                        for k in range(len(json.loads(self.list_msg_tracker[i].src))):
                            source = Tracked_source()
                            source.x = json.loads(self.list_msg_tracker[i].src)[k]["x"]
                            source.y = json.loads(self.list_msg_tracker[i].src)[k]["y"]
                            source.z = json.loads(self.list_msg_tracker[i].src)[k]["z"]
                            source.activity = json.loads(self.list_msg_tracker[i].src)[k]["activity"]

                            for l in range(len(json.loads(self.list_msg_pot[p].src))):
                                pot_x = json.loads(self.list_msg_pot[p].src)[l]["x"]
                                pot_y = json.loads(self.list_msg_pot[p].src)[l]["y"]
                                pot_z = json.loads(self.list_msg_pot[p].src)[l]["z"]
                                pot_E = json.loads(self.list_msg_pot[p].src)[l]["E"]

                                if (source.x - TOL < pot_x < source.x +TOL) and (source.y - TOL < pot_y < source.y +TOL) and (source.z - TOL < pot_z < source.z + TOL):
                                    self.list_E[k][1:] = self.list_E[k][:-1]
                                    self.list_E[k][0] = pot_E
                        
                            source.E = sum(self.list_E[k])/len(self.list_E[k])
                            
                            if source.E == self.last_E[k]:
                                self.reset_list_E[k] = self.reset_list_E[k] + 1
                                if self.reset_list_E[k] > 200:
                                    source.E = 0.00
                                    self.list_E[k] = [0.000]*E_MOY_NUM
                                    self.reset_list_E[k] = 0
                            self.last_E[k] = source.E
                            
                            src = json.dumps(json.loads(self.list_msg_speech[j].src)[k]["category"])
                            src = src[1:-1]
                            if "non" in src:
                                source.speech = False
                            else:
                                source.speech = True
                            msg_.sources.append(source)
                            
                        h = std_msgs.msg.Header()
                        h.stamp = rospy.Time.now()
                        msg_.header = h
                        self.tracked_pub.publish(msg_)
                        self.list_msg_tracker.pop(i)
                        self.list_msg_speech.pop(j)
                        self.list_msg_pot.pop(j)
                        self.MsgMatching()  # Recursive call (not optimal but works well at low list count)
                        return
                    if self.list_msg_tracker[i].timestamp < self.lastTimestamp:
                        self.list_msg_tracker.pop(i)
                        self.MsgMatching()  # Recursive call (not optimal but works well at low list count)
                        return
                    if self.list_msg_speech[j].timestamp < self.lastTimestamp:
                        self.list_msg_speech.pop(j)
                        self.MsgMatching()  # Recursive call (not optimal but works well at low list count)
                        return
                    if self.list_msg_pot[p].timestamp < self.lastTimestamp:
                        self.list_msg_speech.pop(p)
                        self.MsgMatching()  # Recursive call (not optimal but works well at low list count)
                        return


    def socketTracker(self, conn):
        while not rospy.is_shutdown():
            data = conn.recv(8192)
            try:
                #print data
                msg_ = Msg("tracker",data)
                self.list_msg_tracker.append(msg_)
            except ValueError:
                if PRINT : print "odas server not working (Take a look at soundusb card id)"
            self.MsgMatching()
            #msg_.printMsg()
            #print "Tracker len = %d  &&  Speech len = %d" %(len(self.list_msg_tracker), len(self.list_msg_speech))
        conn.close()

    def socketSpeech(self, conn):
        while not rospy.is_shutdown():
            data = conn.recv(8192)
            try:
                #print data
                msg_ = Msg("speech",data)
                self.list_msg_speech.append(msg_)
            except ValueError:
                if PRINT : print "odas server not working (Take a look at soundusb card id)"
            #msg_.printMsg()
            #print "Tracker len = %d  &&  Speech len = %d" %(len(self.list_msg_tracker), len(self.list_msg_speech))
            #self.MsgMatching()
        conn.close()

    def socketPot(self, conn):
        while not rospy.is_shutdown():
            data = conn.recv(8192)
            try:
                #print data
                msg_ = Msg("pot",data)
                self.list_msg_pot.append(msg_)
            except ValueError:
                if PRINT : print "odas server not working (Take a look at soundusb card id)"
            #msg_.printMsg()
            #print "Tracker len = %d  &&  Speech len = %d" %(len(self.list_msg_tracker), len(self.list_msg_speech))
            #self.MsgMatching()
        conn.close()    

    def spin(self):
        while not rospy.is_shutdown():
            connTrack, addrTrack = self.list_socket[0].accept()
            connSpeech, addrSpeech = self.list_socket[1].accept()
            connPot, addrPot = self.list_socket[2].accept()
            if PRINT : print '[*] Connected with ' + addrTrack[0] + ':' + str(addrTrack[1])
            if PRINT : print '[*] Connected with ' + addrSpeech[0] + ':' + str(addrSpeech[1])
            if PRINT : print '[*] Connected with ' + addrPot[0] + ':' + str(addrPot[1])
            start_new_thread(self.socketTracker ,(connTrack,))
            start_new_thread(self.socketSpeech ,(connSpeech,))
            start_new_thread(self.socketPot ,(connPot,))
                
        s.close()


def main(args):
    '''Initializes and cleanup ros node'''
    odas_ = odas()
    rospy.init_node('odas_driver', anonymous=True, disable_signals=True)
    try:
        odas_.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS ODAS driver"

if __name__ == '__main__':
    if PRINT : print "Hello World!\n"
    main(sys.argv)
    if PRINT : print "GoodBye World!\n"
