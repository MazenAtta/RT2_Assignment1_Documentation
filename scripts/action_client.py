#!/usr/bin/env python3

"""
.. module: action_client
   :platform: unix
   :synopsis: Python module for sending goals to an action server.
.. moduleauthor:: MazenAtta

ROS node that acts as an action client for sending goals to an action server.

Subscribes to:
    /odom

Publishes to:
    /reaching_goal/feedback

Clients:
    /reaching_goal
"""

import rospy
import actionlib
from assignment2_rt.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String

# Global variables
robot_position = Point()


def odom_callback(data):
    """
    Callback function to update the robot's position.

    *Important:* This function is called whenever a new message is received on the /odom topic. It updates the global variable `robot_position` with the latest position data.

    Args:
        data (Odometry): The odometry data from the /odom topic, which includes the robot's current position and orientation.
    """
    global robot_position
    robot_position = data.pose.pose.position


def send_goal(client, x, y):
    """
    Send a goal to the action server.

    *Action:* This function creates a new goal with the specified x and y coordinates and sends it to the action server through the action client.

    Args:
        client (SimpleActionClient): The action client used to communicate with the action server.
        x (float): The x-coordinate of the goal.
        y (float): The y-coordinate of the goal.

    Example:
        send_goal(client, 1.0, 2.0)
    """
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    client.send_goal(goal)
    rospy.loginfo(f"Goal sent: x={x}, y={y}")


def cancel_goal(client):
    """
    Cancel the current goal.

    *Action:* This function cancels any active goal that has been sent to the action server.

    Args:
        client (SimpleActionClient): The action client used to communicate with the action server.

    Example:
        cancel_goal(client)
    """
    client.cancel_goal()
    rospy.loginfo("Goal cancelled.")


def main():
    """
    Main function to initialize the node and handle user input.

    *Action:* This function initializes the ROS node, sets up the action client and subscribers, and handles user input to send or cancel goals.

    Workflow:
        1. Initialize the ROS node.
        2. Set up the action client to communicate with the /reaching_goal action server.
        3. Subscribe to the /odom topic to receive odometry data.
        4. Create a publisher for custom feedback messages.
        5. Enter a loop to handle user input for sending or canceling goals.
        6. Monitor the goal's progress and publish feedback messages.

    Example:
        rosrun package_name action_client.py
    """
    rospy.init_node('action_client_node')

    # Action client setup
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Action server is ready.")

    # Subscriber to /odom
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Publisher for custom feedback (if required by the repo structure)
    feedback_pub = rospy.Publisher('/reaching_goal/feedback', String, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        user_input = input("Enter target (x, y) or 'cancel': ")

        if user_input.lower() == 'cancel':
            cancel_goal(client)
        else:
            try:
                x, y = map(float, user_input.split(','))
                send_goal(client, x, y)

                # Monitor the goal's progress
                while not client.wait_for_result(timeout=rospy.Duration(1.0)):
                    feedback_msg = f"Current position: x={robot_position.x:.2f}, y={robot_position.y:.2f}"
                    rospy.loginfo(feedback_msg)
                    feedback_pub.publish(feedback_msg)

                rospy.loginfo("Goal reached!")
            except ValueError:
                rospy.logerr("Invalid input. Please enter valid x, y coordinates or 'cancel'.")

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass