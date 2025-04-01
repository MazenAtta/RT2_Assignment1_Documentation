#!/usr/bin/env python3

"""
.. module: service_node
   :platform: unix
   :synopsis: Python module for providing the last target coordinates as a service.
.. moduleauthor:: MazenAtta

ROS node that provides a service to get the last target coordinates.

Subscribes to:
    /last_target

Services:
    get_target
"""

import rospy
from assignment_2_2024.srv import GetTarget, GetTargetResponse
from geometry_msgs.msg import Point

# Global variable to store the last target
last_target = {"x": 0.0, "y": 0.0}

def handle_get_target(req):
    """
    Handle the service request to get the last target coordinates.

    *Action:* This function is called whenever a service request is received on the /get_target service. It returns the last target coordinates.

    Args:
        req (GetTarget.Request): The service request.

    Returns:
        GetTargetResponse: The response containing the last target coordinates.

    Example:
        response = handle_get_target(request)
    """
    rospy.loginfo(f"Returning last target: ({last_target['x']}, {last_target['y']})")
    return GetTargetResponse(last_target["x"], last_target["y"])

def target_callback(msg):
    """
    Callback function to update the last target coordinates.

    *Action:* This function is called whenever a new message is received on the /last_target topic. It updates the global `last_target` variable.

    Args:
        msg (Point): The message containing the last target coordinates.
    
    Returns:
        None

    """
    global last_target
    last_target["x"] = msg.x
    last_target["y"] = msg.y
    rospy.loginfo(f"Updated last target to: ({last_target['x']}, {last_target['y']})")

def main():
    """
    Main function to initialize the node and handle service requests.

    *Action:* This function initializes the ROS node, sets up the service server, and subscribes to the /last_target topic.

    Workflow:
        1. Initialize the ROS node.
        2. Subscribe to the /last_target topic to receive the last target coordinates.
        3. Advertise the /get_target service to handle service requests.
        4. Keep the node running to handle incoming service requests.

    Example:
        rosrun package_name service_node.py
    """
    rospy.init_node('service_node')

    # Subscribe to /last_target
    rospy.Subscriber('/last_target', Point, target_callback)

    # Advertise the service
    rospy.Service('get_target', GetTarget, handle_get_target)
    rospy.loginfo("Service 'get_target' is ready to provide the last target coordinates.")

    # Keep the node running
    rospy.spin()

if __name__ == "__main__":
    main()