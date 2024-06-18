#! /usr/bin/env python3

import rclpy

from odas_ros.lib_odas_server_node import OdasServerNode


def main():
    rclpy.init()
    odas_server_node = OdasServerNode('odas_server_node')

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
