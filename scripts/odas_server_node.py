#!/usr/bin/env python2

import json
import socket
import threading

import libconf
import io

import rospy

from std_msgs.msg import Header
from odas_ros.msg import OdasSst, OdasSstArrayStamped
from audio_utils.msg import AudioFrame


class OdasServerNode:
    def __init__(self):
        self._configuration = self._load_configuration(rospy.get_param('~configuration_path'))
        self._sst_frame_id = rospy.get_param('~sst_frame_id')

        self._verify_sst_configuration()
        self._verify_sss_configuration()

        self._sst_port = self._configuration['sst']['tracked']['interface']['port']
        
        self._sss_port = self._configuration['sss']['separated']['interface']['port']
        self._sss_nbits = self._configuration['sss']['separated']['nBits']
        self._sss_format = self._sss_nbits_to_sss_format(self._sss_nbits)
        self._sss_channel_count = len(self._configuration['sst']['N_inactive'])
        self._sss_sampling_frequency = self._configuration['sss']['separated']['fS']
        self._sss_frame_sample_count = self._configuration['sss']['separated']['hopSize']

        self._sst_server_socket = None
        self._sst_client_socket = None
        self._sss_server_socket = None
        self._sss_client_socket = None

        self._sst_pub = rospy.Publisher('sst', OdasSstArrayStamped, queue_size=10)
        self._sss_pub = rospy.Publisher('sss', AudioFrame, queue_size=10)

    def _load_configuration(self, configuration_path):
        with io.open(configuration_path) as f:
            return libconf.load(f)

    def _verify_sst_configuration(self):
        if self._configuration['sst']['tracked']['format'] != 'json' or self._configuration['sst']['tracked']['interface']['type'] != 'socket':
			raise ValueError('The sst format must be "json" and the sst interface type must be "socket"')

    def _verify_sss_configuration(self):
        if self._configuration['sss']['separated']['interface']['type'] != 'socket':
			raise ValueError('The sss interface type must be "socket"')


    def _sss_nbits_to_sss_format(self, nbits):
        if nbits == 8:
            return 'signed_8'
        elif nbits == 16:
            return 'signed_16'
        elif nbits == 32:
            return 'signed_32'
        else:
            raise ValueError('Not supported format (nbits={})'.format(nbits))

    def _create_server_socket(self, port):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket. SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('', port))
        server_socket.listen(5)
        server_socket.settimeout(0.1)

        return server_socket

    def _sst_thread_run(self):
        self._sst_server_socket = self._create_server_socket(self._sst_port)
        recv_size = 8192

        while not rospy.is_shutdown():
            try:
                self._sst_client_socket, _ = self._sst_server_socket.accept()
            except socket.timeout:
                continue

            while not rospy.is_shutdown():
                data = self._sst_client_socket.recv(recv_size)
                if not data:
                    break

                data = data.decode('utf-8')
                messages = data.split(']\n}\n')

                for message in messages:
                    message += ']\n}\n'
                    try:
                        sst = json.loads(data)
                        self._send_sst(sst)
                    #except json.JSONDecodeError:
                    except Exception as e:
			print(e)
                        continue

    def _send_sst(self, sst):
        odas_sst_array_stamped_msg = OdasSstArrayStamped()
        odas_sst_array_stamped_msg.header.seq = sst['timeStamp']
        odas_sst_array_stamped_msg.header.stamp = rospy.Time.now()
        odas_sst_array_stamped_msg.header.frame_id = self._sst_frame_id

        for source in sst['src']:
            if source['id'] != 0:
                odas_sst = OdasSst()
                odas_sst.id = source['id']
                odas_sst.x = source['x']
                odas_sst.y = source['y']
                odas_sst.z = source['z']
                odas_sst.activity = source['activity']
                odas_sst_array_stamped_msg.sources.append(odas_sst)

        self._sst_pub.publish(odas_sst_array_stamped_msg)

    def _sss_thread_run(self):
        self._sss_server_socket = self._create_server_socket(self._sss_port)
        recv_size = self._sss_nbits // 8 * self._sss_channel_count * self._sss_frame_sample_count

        while not rospy.is_shutdown():
            try:
                self._sss_client_socket, _ = self._sss_server_socket.accept()
            except socket.timeout:
                continue

            while not rospy.is_shutdown():
                data = self._sss_client_socket.recv(recv_size)
                if not data:
                    break
                elif len(data) != recv_size:
                    continue
                else:
                    self._send_sss(data)

    def _send_sss(self, data):
        audio_frame_msg = AudioFrame()
        audio_frame_msg.format = self._sss_format
        audio_frame_msg.channel_count = self._sss_channel_count
        audio_frame_msg.sampling_frequency = self._sss_sampling_frequency
        audio_frame_msg.frame_sample_count = self._sss_frame_sample_count
        audio_frame_msg.data = data
        self._sss_pub.publish(audio_frame_msg)

    def run(self):
        sst_thread = threading.Thread(target=self._sst_thread_run)
        sss_thread = threading.Thread(target=self._sss_thread_run)

        sst_thread.start()
        sss_thread.start()

        rospy.spin()

        self._sst_server_socket.close()
        self._sss_server_socket.close()

        if self._sst_client_socket is not None:
            self._sst_client_socket.close()
        if self._sss_client_socket is not None:
            self._sss_client_socket.close()

        sst_thread.join()
        sss_thread.join()


def main():
    rospy.init_node('odas_server_node')
    odas_server_node = OdasServerNode()
    odas_server_node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
