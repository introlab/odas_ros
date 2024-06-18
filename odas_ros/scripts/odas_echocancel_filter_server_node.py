#! /usr/bin/env python3

import rclpy
import numpy as np

from odas_ros.lib_odas_server_node import OdasServerNode, OdasSstArrayStamped
from audio_utils import AudioFrame, convert_audio_data_to_numpy_frames, get_format_information


class OdasFilterEchocancelServerNode(OdasServerNode):

    def __init__(self):
        super().__init__('odas_server_node')
        self.energy = None
        self.inertia = 0.0

        self._ec_signal_sub = self.create_subscription(AudioFrame, 'ec_signal', self._ec_signal_callback, 10)
        self._energy_threshold = self.declare_parameter('energy_threshold', 0.5).get_parameter_value().double_value
        self._energy_inertia = self.declare_parameter('energy_inertia', 20).get_parameter_value().integer_value

    @staticmethod
    def compute_energy(frame: AudioFrame) -> np.ndarray:
        frames = convert_audio_data_to_numpy_frames(
            get_format_information(frame.format), frame.channel_count, frame.data)
        energy = np.array([sum(value*value for value in channel)
                          for channel in frames])
        return energy

    def _ec_signal_callback(self, msg):
        self.energy = self.compute_energy(msg)
        self.inertia += 1 * (self.should_send_energy and
                             self.inertia != self._energy_inertia)
        self.inertia -= 1 * (not self.should_send_energy and self.inertia != 0)

    @property
    def should_send_energy(self) -> bool:
        return self.energy is None or np.any(self.energy >= self._energy_threshold)

    def _send_sst(self, sst):
        if self.should_send_energy or self.inertia != 0:
            super()._send_sst(sst)
        else:
            self._send_empty_sst_array(sst)

    def _send_empty_sst_array(self, sst):
        odas_sst_array_stamped_msg = OdasSstArrayStamped()
        odas_sst_array_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        odas_sst_array_stamped_msg.header.frame_id = self._frame_id

        self._sst_pub.publish(odas_sst_array_stamped_msg)


def main():
    rclpy.init()
    odas_server_node = OdasFilterEchocancelServerNode()

    try:
        odas_server_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        odas_server_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
