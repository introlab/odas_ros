#! /usr/bin/env python3

import rospy
from odas_ros.lib_odas_server_node import OdasServerNode


def main():
    rospy.init_node('odas_server_node')
    odas_server_node = OdasServerNode()
    odas_server_node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
