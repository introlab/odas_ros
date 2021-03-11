#!/usr/bin/env python2

import rospy

import tf2_geometry_msgs, tf2_ros, tf_conversions
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

from odas_ros.msg import OdasSst, OdasSstArrayStamped, OdasSsl, OdasSslArrayStamped
from sensor_msgs.msg import PointCloud2, PointField

class OdasVisualizationNode:
    def __init__(self):
        self._configuration = self._load_configuration(rospy.get_param('~configuration_path'))

        # Get enable parameters for SSL, SST and SSS (Sound Source Localization, Tracking and Separation)
        self._ssl_enabled = rospy.get_param('~ssl_enabled')
        self._sst_enabled = rospy.get_param('~sst_enabled')

        if self._sst_enabled == "true":
            # Stamped Pose Message containing the converted Sound Source Tracking (SST) position from ODAS.
            self._sst_input_PoseStamped = tf2_geometry_msgs.PoseStamped()
            self._sst_input_PoseStamped.pose.position.x = 0
            self._sst_input_PoseStamped.pose.position.y = 0
            self._sst_input_PoseStamped.pose.position.z = 0
            # Subscribe to the Sound Source Tracking from ODAS Server
            self._sst_sub = rospy.Subscriber('sst', OdasSstArrayStamped, self._sst_cb, queue_size=10)
            # ODAS SST Publisher for PoseStamped
            self._sst_pose_pub = rospy.Publisher('sst_pose', tf2_geometry_msgs.PoseStamped, queue_size=1)

        if self._ssl_enabled == "true":
            # Subscribe to the Sound Source Localization and Sound Source Tracking from ODAS Server
            self._ssl_sub = rospy.Subscriber('ssl', OdasSslArrayStamped, self._ssl_cb, queue_size=10)
            # ODAS SSL Publisher for PointCloud2
            self._ssl_pcl_pub = rospy.Publisher("ssl_pcl2", PointCloud2, queue_size=500)

    def _ssl_cb(self, ssl):
        # Sound Source Localization Callback (ODAS)
        if len(ssl.sources) == 0:
            return

        if len(ssl.sources) > 0:
            cloud_points = []
            for source in ssl.sources:
                # Extract xyz position of potential sound source on unit sphere from Sound Source Localization
                point = [source.x, source.y, source.z, source.E]
                cloud_points.append(point)
            #header
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = ssl.header.frame_id
            #fields
            fields = [ PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1),
                       PointField('intensity', 12, PointField.FLOAT32, 1)]
            #create pcl from points
            pcl = pcl2.create_cloud(header,fields,cloud_points)
            self._ssl_pcl_pub.publish(pcl)

    def _sst_cb(self, sst):
        # Sound Source Tracking Callback (ODAS)
        if len(sst.sources) == 0:
            return

        if len(sst.sources) > 1:
            rospy.logerr('Invalid sst (len(sst.sources)={})'.format(len(sst.sources)))
            return

        # Extract xyz position of source on unit sphere from Sound Source Tracking
        x = sst.sources[0].x
        y = sst.sources[0].y
        z = sst.sources[0].z

        # Convert the xyz position of the unit vector in quaternion
        q = self._unitVector_to_quaternion(x,y,z)

        # Update the SST PoseStamped
        self._sst_input_PoseStamped.pose.orientation.x = q[0]
        self._sst_input_PoseStamped.pose.orientation.y = q[1]
        self._sst_input_PoseStamped.pose.orientation.z = q[2]
        self._sst_input_PoseStamped.pose.orientation.w = q[3]
        self._sst_input_PoseStamped.header.frame_id = sst.header.frame_id
        self._sst_input_PoseStamped.header.stamp = rospy.Time.now()

        self._sst_pose_pub.publish(self._transform_pose(self._sst_input_PoseStamped, "odas_link"))

    def _unitVector_to_quaternion(self, x, y, z):
        # Convert a xyz unit vector (point on a unit sphere) to quaternion
        yaw = np.arctan2(y,x)
        pitch = -np.arctan2(z,np.sqrt(x*x+y*y))
        roll = 0
        q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
        return q

    def run(self):
        rospy.spin()

def main():
    rospy.init_node('odas_visualization_node')
    odas_visualization_node = OdasVisualizationNode()
    odas_visualization_node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
