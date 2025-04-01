#!/usr/bin/env python3

"""
.. module: service_node
   :platform: unix
   :synopsis: Python module for providing the last target coordinates as a service.
.. moduleauthor:: MazenAtta mazenatta445@gmail.com

ROS node that provides a service to get the last target coordinates.

Clients:
    /get_last_target
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger  # Modify this if using a custom service
from geometry_msgs.msg import Pose  # For target position

class TargetServiceNode(Node):
    """
    A ROS node that provides the last target coordinates service.

    Attributes:
        srv (Service): The service server instance.
        last_target (Pose): The last target coordinates.
    
    """
    def __init__(self):
        super().__init__('target_service_node')
        self.srv = self.create_service(Trigger, 'get_last_target', self.get_last_target_callback)
        self.last_target = Pose()
        self.get_logger().info("Service Node ready to provide the last target coordinates.")

    def set_last_target(self, x, y):
        """
        Update the last target with new coordinates.

        Args:
            x (float): The x-coordinate of the target.
            y (float): The y-coordinate of the target.

        Returns:
        None

        """
        self.last_target.position.x = x
        self.last_target.position.y = y

    def get_last_target_callback(self, request, response):
        """
        Callback function to handle service requests.

        Args:
            request (Trigger.Request): The service request.
            response (Trigger.Response): The service response.

        Returns:
            Trigger.Response: The populated service response.
        """
        response.success = True
        response.message = (
            f"Last target coordinates: x={self.last_target.position.x}, "
            f"y={self.last_target.position.y}"
        )
        self.get_logger().info(response.message)
        return response


def main(args=None):
    """
    Main function to initialize the node and spin.

    Args:
    None

    Returns:
    None

    """
    rclpy.init(args=args)
    node = TargetServiceNode()
    node.set_last_target(2.0, 3.0)  # Example target coordinates
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()