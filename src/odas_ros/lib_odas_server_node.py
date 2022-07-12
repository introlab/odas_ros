from abc import ABC, abstractmethod
import json
import socket
import threading
import os
import subprocess
import queue
from typing import ByteString

import libconf
import io

import rospy

from odas_ros.msg import OdasSst, OdasSstArrayStamped, OdasSsl, OdasSslArrayStamped
from audio_utils.msg import AudioFrame


RAW_QUEUE_SIZE = 100


def nbits_to_format(nbits):
    if nbits == 8:
        return 'signed_8'
    elif nbits == 16:
        return 'signed_16'
    elif nbits == 32:
        return 'signed_32'
    else:
        raise ValueError('Not supported format (nbits={})'.format(nbits))


class SocketServer(ABC):
    def __init__(self, port: int):
        rospy.loginfo("Creating server socket on port: " + str(port))
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket. SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind(('', port))
        self._server_socket.listen(5)
        self._server_socket.settimeout(0.1)

        self._thread = threading.Thread(target=self._run)
        self._is_stopped = True

    def start(self):
        self._is_stopped = False
        self._thread.start()

    def close(self):
        self._is_stopped = True
        self._server_socket.close()
        self._thread.join()

    def _run(self):
         while not self._is_stopped:
            try:
                client_socket, _ = self._server_socket.accept()
            except (socket.timeout, OSError):
                continue

            try:
                self._handle_client(client_socket)
            finally:
                client_socket.close()

    @abstractmethod
    def _handle_client(self, client_socket):
        pass


class RawSocketServer(SocketServer):
    def __init__(self, configuration: dict, audio_frame_timestamp_queue: queue.Queue):
        super().__init__(configuration['raw']['interface']['port'])
        self._audio_frame_timestamp_queue = audio_frame_timestamp_queue

        self._raw_nbits = configuration['raw']['nBits']
        self._raw_format = nbits_to_format(self._raw_nbits)
        self._raw_channel_count = configuration['raw']['nChannels']
        self._raw_sampling_frequency = configuration['raw']['fS']
        self._raw_frame_sample_count = configuration['raw']['hopSize']

        self._raw_queue = queue.Queue(maxsize=RAW_QUEUE_SIZE)
        self._raw_sub = rospy.Subscriber('raw', AudioFrame, self._raw_audio_cb, queue_size=RAW_QUEUE_SIZE)

    def _raw_audio_cb(self, msg: AudioFrame):
        if (msg.format != self._raw_format or
            msg.channel_count != self._raw_channel_count or
            msg.sampling_frequency != self._raw_sampling_frequency or
            msg.frame_sample_count != self._raw_frame_sample_count):
            rospy.logerr(
                'Invalid frame (msg.format={}, msg.channel_count={}, msg.sampling_frequency={}, msg.frame_sample_count={})'
                .format(msg.format, msg.channel_count, msg.sampling_frequency, msg.frame_sample_count))
            return

        if self._audio_frame_timestamp_queue is not None:
            self._audio_frame_timestamp_queue.put(msg.header.stamp)
        self._raw_queue.put(msg.data)

    def close(self):
        self._is_stopped = True
        self._raw_queue.put(None)
        super().close()

    def _handle_client(self, client_socket: socket.socket):
        while not self._is_stopped:
            data = self._raw_queue.get()
            if data is None or client_socket.send(data) == 0:
                break


class JsonSocketServer(SocketServer):
    def _handle_client(self, client_socket: socket.socket):
        recv_size = 8192
        while not self._is_stopped:
            data = client_socket.recv(recv_size)
            if not data:
                break

            data = data.decode('utf-8')
            messages = data.split(']\n}\n')
            messages.pop()  # remove last item which is only a newline character

            for message in messages:
                message += ']\n}\n'
                try:
                    data = json.loads(message)
                    self._handle_data(data)
                except Exception as e:
                    rospy.logerr(e)
                    continue

    @abstractmethod
    def _handle_data(self, data):
        pass


class SslSocketServer(JsonSocketServer):
    def __init__(self, configuration: dict, frame_id: str):
        super().__init__(configuration['ssl']['potential']['interface']['port'])
        self._frame_id = frame_id
        self._ssl_pub = rospy.Publisher('ssl', OdasSslArrayStamped, queue_size=10)

    def _handle_data(self, ssl: dict):
        odas_ssl_array_stamped_msg = OdasSslArrayStamped()
        odas_ssl_array_stamped_msg.header.seq = ssl['timeStamp']
        odas_ssl_array_stamped_msg.header.stamp = rospy.Time.now()
        odas_ssl_array_stamped_msg.header.frame_id = self._frame_id

        for source in ssl['src']:
            odas_ssl = OdasSsl()
            odas_ssl.x = source['x']
            odas_ssl.y = source['y']
            odas_ssl.z = source['z']
            odas_ssl.E = source['E']
            odas_ssl_array_stamped_msg.sources.append(odas_ssl)

        self._ssl_pub.publish(odas_ssl_array_stamped_msg)


class SstSocketServer(JsonSocketServer):
    def __init__(self, configuration: dict, frame_id: str):
        super().__init__(configuration['sst']['tracked']['interface']['port'])
        self._frame_id = frame_id
        self._sst_pub = rospy.Publisher('sst', OdasSstArrayStamped, queue_size=10)

    def _handle_data(self, sst: dict):
        odas_sst_array_stamped_msg = OdasSstArrayStamped()
        odas_sst_array_stamped_msg.header.seq = sst['timeStamp']
        odas_sst_array_stamped_msg.header.stamp = rospy.Time.now()
        odas_sst_array_stamped_msg.header.frame_id = self._frame_id

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


class SssSocketServer(SocketServer):
    def __init__(self, configuration: dict, audio_frame_timestamp_queue: queue.Queue, frame_id: str):
        super().__init__(configuration['sss']['separated']['interface']['port'])
        self._audio_frame_timestamp_queue = audio_frame_timestamp_queue
        self._frame_id = frame_id

        self._sss_nbits = configuration['sss']['separated']['nBits']
        self._sss_format = nbits_to_format(self._sss_nbits)
        self._sss_channel_count = len(configuration['sst']['N_inactive'])
        self._sss_sampling_frequency = configuration['sss']['separated']['fS']
        self._sss_frame_sample_count = configuration['sss']['separated']['hopSize']

        self._sss_pub = rospy.Publisher('sss', AudioFrame, queue_size=100)

    def _handle_client(self, client_socket: socket.socket):
        recv_size = recv_size = self._sss_nbits // 8 * self._sss_channel_count * self._sss_frame_sample_count

        while not self._is_stopped:
            data = client_socket.recv(recv_size)
            if not data:
                break
            elif len(data) != recv_size:
                continue
            else:
                self._send_sss(data)

    def _send_sss(self, data: ByteString):
        audio_frame_msg = AudioFrame()
        audio_frame_msg.header.stamp = self._get_timestamp()
        audio_frame_msg.header.frame_id = self._frame_id
        audio_frame_msg.format = self._sss_format
        audio_frame_msg.channel_count = self._sss_channel_count
        audio_frame_msg.sampling_frequency = self._sss_sampling_frequency
        audio_frame_msg.frame_sample_count = self._sss_frame_sample_count
        audio_frame_msg.data = data
        self._sss_pub.publish(audio_frame_msg)

    def _get_timestamp(self):
        if self._audio_frame_timestamp_queue is None:
            return rospy.Time.now()
        else:
            return self._audio_frame_timestamp_queue.get()


class OdasServerNode:
    def __init__(self):
        self._configuration = self._load_configuration(rospy.get_param('~configuration_path'))
        frame_id = rospy.get_param('~frame_id')

        if self._verify_raw_and_sss_configuration():
            audio_frame_timestamp_queue = queue.Queue()
        else:
            audio_frame_timestamp_queue = None

        if self._verify_raw_configuration():
            self._raw_socket_server = RawSocketServer(self._configuration, audio_frame_timestamp_queue)
        else:
            self._raw_socket_server = None

        if self._verify_ssl_configuration():
            self._ssl_socket_server = SslSocketServer(self._configuration, frame_id)
        else:
            self._ssl_socket_server = None

        if self._verify_sst_configuration():
            self._sst_socket_server = SstSocketServer(self._configuration, frame_id)
        else:
            self._sst_socket_server = None

        if self._verify_sss_configuration():
            self._sss_socket_server = SssSocketServer(self._configuration, audio_frame_timestamp_queue, frame_id)
        else:
            self._sss_socket_server = None

    def _load_configuration(self, configuration_path: str):
        with io.open(configuration_path) as f:
            return libconf.load(f)

    def _verify_raw_configuration(self):
        return self._configuration['raw']['interface']['type'] == 'socket'

    def _verify_ssl_configuration(self):
        if self._configuration['ssl']['potential']['interface']['type'] != 'socket':
            return False
        elif self._configuration['ssl']['potential']['format'] != 'json':
            raise ValueError('The ssl format must be "json"')
        else:
            return True

    def _verify_sst_configuration(self):
        if self._configuration['sst']['tracked']['interface']['type'] != 'socket':
            return False
        elif self._configuration['sst']['tracked']['format'] != 'json':
            raise ValueError('The sst format must be "json"')
        else:
            return True

    def _verify_sss_configuration(self):
        return self._configuration['sss']['separated']['interface']['type'] == 'socket'

    def _verify_raw_and_sss_configuration(self):
        if (self._configuration['raw']['interface']['type'] != 'socket' or
            self._configuration['sss']['separated']['interface']['type'] != 'socket'):
            return False

        if self._configuration['raw']['fS'] != self._configuration['sss']['separated']['fS']:
            raise ValueError('Raw and sss sampling frequencies must match.')
        if self._configuration['raw']['hopSize'] != self._configuration['sss']['separated']['hopSize']:
            raise ValueError('Raw and sss hop sizes must match.')

        return True

    def run(self):
        if self._raw_socket_server:
            self._raw_socket_server.start()
            rospy.loginfo("Raw socket server started")
        if self._ssl_socket_server:
            self._ssl_socket_server.start()
            rospy.loginfo("Sound Source Localization socket server started")
        if self._sst_socket_server:
            self._sst_socket_server.start()
            rospy.loginfo("Sound Source Tracking socket server started")
        if self._sss_socket_server:
            self._sss_socket_server.start()
            rospy.loginfo("Sound Source Separation socket server started")

        executable_args = ["rosrun",
                           "odas_ros",
                           "odas_core_node",
                           "_configuration_path:=" + rospy.get_param('~configuration_path')]

        odas_core_process = subprocess.Popen(executable_args, cwd=os.curdir)

        rospy.spin()

        odas_core_process.terminate()

        if self._raw_socket_server:
            self._raw_socket_server.close()
        if self._ssl_socket_server:
            self._ssl_socket_server.close()
        if self._sst_socket_server:
            self._sst_socket_server.close()
        if self._sss_socket_server:
            self._sss_socket_server.close()
