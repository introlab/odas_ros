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
    #Class containning the information of the messages read on the sockets json sent from odas
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
        # important variables
        self.list_socket = []
        self.nbr_socket = 3
        self.list_msg_tracker = []
        self.list_msg_speech = []
        self.list_msg_pot = []
        self.lastTimestamp = 0
        self.list_E = [0]*NUM_SOURCE
        self.raise_incomplete_Tracker = False
        self.raise_incomplete_Speech = False
        self.raise_incomplete_Pot = False
        self.remaining_Tracker = []
        self.remaining_Speech = []
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

    def MsgMatching(self):
    # function that uses the list_msg of every socket and uses them to either publish the data or erase it
        for i in range(len(self.list_msg_tracker)):
            for j in range(len(self.list_msg_speech)):
                for p in range(len(self.list_msg_pot)):
                    if self.list_msg_tracker[i].timestamp == self.list_msg_speech[j].timestamp == self.list_msg_pot[p].timestamp:
                        self.lastTimestamp = self.list_msg_speech[j].timestamp
                        msg_ = Tracked_sources()
                        for k in range(len(json.loads(self.list_msg_tracker[i].src))):
                            # When every timestamps are good, we load the information from the msg into a publishable variable
                            source = Tracked_source()
                            source.x = json.loads(self.list_msg_tracker[i].src)[k]["x"]
                            source.y = json.loads(self.list_msg_tracker[i].src)[k]["y"]
                            source.z = json.loads(self.list_msg_tracker[i].src)[k]["z"]
                            source.activity = json.loads(self.list_msg_tracker[i].src)[k]["activity"]
                            
                            # For every source in the pot message, in order of most energy to least
                            for l in range(len(json.loads(self.list_msg_pot[p].src))):
                                pot_x = json.loads(self.list_msg_pot[p].src)[l]["x"]
                                pot_y = json.loads(self.list_msg_pot[p].src)[l]["y"]
                                pot_z = json.loads(self.list_msg_pot[p].src)[l]["z"]
                                pot_E = json.loads(self.list_msg_pot[p].src)[l]["E"]
                                
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
                            
                            # Looking for speech if it is true or not using the returned nonspeech or speech from odas_ros
                            src = json.dumps(json.loads(self.list_msg_speech[j].src)[k]["category"])
                            src = src[1:-1]
                            if "non" in src:
                                source.speech = False
                            else:
                                source.speech = True
                            msg_.sources.append(source)
                        
                        # Publish the msg_ containning all the source information    
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
                    self.list_msg_tracker.append(msg_)
                if len(test[-1]) > 0:
                    #print "Oh oh"
                    self.raise_incomplete_Tracker = True
                    self.remaining_Tracker.append(test[-1])
                    #print 'what is remaining: ', self.remaining_Tracker
            except ValueError:
                if PRINT : print "odas server not working socket Tracker(Take a look at soundusb card id)"
            self.MsgMatching()
            #msg_.printMsg()
            
        conn.close()

    def socketSpeech(self, conn):
    # Function receiving information from the odas speech socket and making sure it gets into a 
    # Msg class without being corrupted by the reading of the socket.
        while not rospy.is_shutdown():
            data = conn.recv(8192)
            test = data.split(']\n}\n')
            try:
                if self.raise_incomplete_Speech:
                    self.raise_incomplete_Speech = False
                    test[0] = self.remaining_Speech[0] + test[0]
                    self.remaining_Speech.pop(0)
                for message in test[0:-1:1]:
                    message = message + ' ]\n}\n'
                    #print data
                    msg_ = Msg("speech",message)
                    self.list_msg_speech.append(msg_)
                if len(test[-1]) > 0:
                    self.raise_incomplete_Speech=True
                    self.remaining_Speech.append(test[-1])
            except ValueError:
                if PRINT : print "odas server not working socket Speech (Take a look at soundusb card id)"
            #self.MsgMatching()
            #msg_.printMsg()
           
        conn.close()

    def socketPot(self, conn):
    # Function receiving information from the odas potential socket and making sure it gets into a 
    # Msg class without being corrupted by the reading of the socket.
        while not rospy.is_shutdown():
            data = conn.recv(8192)
            test = data.split(']\n}\n')
            try:
                if self.raise_incomplete_Pot:
                    self.raise_incomplete_Pot = False
                    test[0] = self.remaining_Pot[0] + test[0]
                    self.remaining_Pot.pop(0)
                for message in test[0:-1:1]:
                    message = message + ']\n}\n'
                    #print data
                    msg_ = Msg("pot",message)
                    self.list_msg_pot.append(msg_)
                if len(test[-1]) > 0:
                    self.raise_incomplete_Pot = True
                    self.remaining_Pot.append(test[-1])
            except ValueError:
                if PRINT : print "odas server not working socket Pot (Take a look at soundusb card id)"
            #self.MsgMatching()
            #msg_.printMsg()
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
