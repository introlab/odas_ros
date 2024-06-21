#!/usr/bin/env python3
import math

import rclpy
import rclpy.node

import numpy as np
import io
import libconf

import std_msgs.msg

import sensor_msgs.point_cloud2 as pcl2  # type: ignore
from odas_ros_msgs.msg import OdasSstArrayStamped, OdasSslArrayStamped
from  geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import PointCloud2, PointField


# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
# the way that ROS prefers it.
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class OdasVisualizationNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('odas_visualization_node')

        # Load ODAS configuration
        configuration_path = self.declare_parameter('configuration_path', '').get_parameter_value().string_value
        self._configuration = self._load_configuration(configuration_path)

        if self._verify_sst_configuration():
            # Stamped Pose Message containing the converted Sound Source Tracking (SST) position from ODAS.
            self._sst_input_PoseArray = PoseArray()
            # Subscribe to the Sound Source Tracking from ODAS Server
            self._sst_sub = self.create_subscription(OdasSstArrayStamped, 'sst', self._sst_cb, 10)
            # ODAS SST Publisher for PoseStamped
            self._sst_pose_pub = self.create_publisher(PoseArray, 'sst_poses', 10)

        if self._verify_ssl_configuration():
            # Subscribe to the Sound Source Localization and Sound Source Tracking from ODAS Server
            self._ssl_sub = self.create_subscription(OdasSslArrayStamped, 'ssl', self._ssl_cb, 10)
            # ODAS SSL Publisher for PointCloud2
            self._ssl_pcl_pub = self.create_publisher(PointCloud2, 'ssl_pcl2', 500)


    def _load_configuration(self, configuration_path):
        with io.open(configuration_path) as f:
            return libconf.load(f)


    def _verify_ssl_configuration(self):
        # If interface type is not socket, SSL disabled.
        # If interface type is socket and the format is json, SSL enabled.
        if self._configuration['ssl']['potential']['interface']['type'] != 'socket':
            return False
        elif self._configuration['ssl']['potential']['format'] != 'json':
            raise ValueError('The ssl format must be "json"')
        else:
            return True


    def _verify_sst_configuration(self):
        # If interface type is not socket, SST disabled.
        # If interface type is socket and the format is json, SST enabled.
        if self._configuration['sst']['tracked']['interface']['type'] != 'socket':
            return False
        elif self._configuration['sst']['tracked']['format'] != 'json':
            raise ValueError('The sst format must be "json"')
        else:
            return True


    def _ssl_cb(self, ssl):
        # Sound Source Localization Callback (ODAS)
        cloud_points = []
        for source in ssl.sources:
            # Extract xyz position of potential sound source on unit sphere from Sound Source Localization
            point = [source.x, source.y, source.z, source.e]
            cloud_points.append(point)
        #header
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = ssl.header.frame_id
        #fields
        fields = [self._point_field('x', 0, PointField.FLOAT32, 1),
                  self._point_field('y', 4, PointField.FLOAT32, 1),
                  self._point_field('z', 8, PointField.FLOAT32, 1),
                  self._point_field('intensity', 12, PointField.FLOAT32, 1)]
        #create pcl from points
        pcl = pcl2.create_cloud(header, fields, cloud_points)
        self._ssl_pcl_pub.publish(pcl)

    def _point_field(self, name, offset, datatype, count):
        msg = PointField()
        msg.name = name
        msg.offset = offset
        msg.datatype = datatype
        msg.count = count
        return msg

    def _sst_cb(self, sst):
        # Sound Source Tracking Callback (ODAS)
        self._sst_input_PoseArray.header.stamp = self.get_clock().now().to_msg()
        self._sst_input_PoseArray.header.frame_id = sst.header.frame_id
        self._sst_input_PoseArray.poses = []

        for src in sst.sources:
            q = self._unit_vector_to_quaternion(src.x, src.y, src.z)

            # Update the SST PoseStamped
            _sst_input_Pose = Pose()
            _sst_input_Pose.position.x = 0.0
            _sst_input_Pose.position.y = 0.0
            _sst_input_Pose.position.z = 0.0
            _sst_input_Pose.orientation.x = q[0]
            _sst_input_Pose.orientation.y = q[1]
            _sst_input_Pose.orientation.z = q[2]
            _sst_input_Pose.orientation.w = q[3]

            self._sst_input_PoseArray.poses.append(_sst_input_Pose)

        self._sst_pose_pub.publish(self._sst_input_PoseArray)


    def _unit_vector_to_quaternion(self, x, y, z):
        # Convert a xyz unit vector (point on a unit sphere) to quaternion
        yaw = np.arctan2(y,x)
        pitch = -np.arctan2(z,np.sqrt(x*x+y*y))
        roll = 0
        q = quaternion_from_euler(roll, pitch, yaw)
        return q


    def run(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    odas_visualization_node = OdasVisualizationNode()

    try:
        odas_visualization_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        odas_visualization_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
