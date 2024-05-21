#! /usr/bin/env python3

import rclpy

from odas_ros.lib_odas_server_node import OdasServerNode


def main():
    rclpy.init()

    odas_server_node = OdasServerNode('odas_server_node')
    odas_server_node.run()

    odas_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
