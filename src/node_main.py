#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from debug_helper_node import DebugHelperNode

def main(args=None):
    rclpy.init(args=args)
    node = DebugHelperNode()
    # while rclpy.ok():
    #     rclpy.spin_once(node)
    #     node.check_for_quit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()