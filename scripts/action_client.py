#!/usr/bin/env python3

"""
.. module: action_client
   :platform: unix
   :synopsis: Python module for sending goals to an action server and publishing robot state.
.. moduleauthor:: MazenAtta

ROS node that acts as an action client for sending goals to an action server, publishing robot state, and last target.

Subscribes to:
    /odom

Publishes to:
    /robot_state
    /last_target

Clients:
    /reaching_goal
"""

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import RobotState  # Custom message
from geometry_msgs.msg import Point  # To publish the last target

class ActionClientNode:
    """
    A Class that acts as an action client for sending, canceling goals to an action server.

    Attributes:
        client (SimpleActionClient): The action client instance.
        state_pub (Publisher): Publisher for the robot state.
        robot_state (RobotState): The current state of the robot.
        last_target_pub (Publisher): Publisher for the last target position.
    """
    def __init__(self):
        rospy.init_node('action_client_node')
        
        # Action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()
        rospy.loginfo("Action server connected!")

        # Publisher for robot state
        self.state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)
        self.robot_state = RobotState()

        # Publisher for last target
        self.last_target_pub = rospy.Publisher('/last_target', Point, queue_size=10)

        # Subscriber for odometry
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        """
        Callback function to update the robot's state.

        *Action:* This function is called whenever a new message is received on the /odom topic. It updates and publishes the robot's state.

        Args:
            msg (Odometry): The odometry data from the /odom topic.
        
        Returns:
            None

        """
        self.robot_state.x = msg.pose.pose.position.x
        self.robot_state.y = msg.pose.pose.position.y
        self.robot_state.vel_x = msg.twist.twist.linear.x
        self.robot_state.vel_z = msg.twist.twist.angular.z
        self.state_pub.publish(self.robot_state)

    def send_goal(self, x, y):
        """
        Send a goal to the action server.

        *Action:* This function creates a new goal with the specified x and y coordinates, publishes the last target, and sends it to the action server.

        Args:
            x (float): The x-coordinate of the goal.
            y (float): The y-coordinate of the goal.
        
        Returns:
            None

        Example:
            send_goal(1.0, 2.0)
        """
        # Publish last target to the /last_target topic
        target_point = Point(x=x, y=y, z=0.0)
        self.last_target_pub.publish(target_point)

        # Send the goal to the action server
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal)
        rospy.loginfo(f"Goal sent: ({x}, {y})")

    def feedback_callback(self, feedback):
        """
        Callback function to handle feedback from the action server.

        *Action:* This function logs feedback received from the action server.

        Args:
            feedback: The feedback data from the action server.

        Returns:
            None

        """
        rospy.loginfo(f"Feedback received: {feedback}")

    def cancel_goal(self):
        """
        Cancel the current goal.

        *Action:* This function cancels any active goal that has been sent to the action server.

        Args:
            None

        Returns:
            None

        Example:
            cancel_goal()
        """
        self.client.cancel_goal()
        rospy.loginfo("Goal canceled")

    def run(self):
        """
        Main function to run the node and handle user input.

        *Action:* This function initializes the ROS node, sets up the action client and subscribers, and handles user input to send or cancel goals.

        Workflow:
            1. Initialize the ROS node.
            2. Set up the action client to communicate with the /reaching_goal action server.
            3. Subscribe to the /odom topic to receive odometry data.
            4. Create publishers for robot state and last target.
            5. Enter a loop to handle user input for sending or canceling goals.

        Example:
            rosrun package_name action_client.py
        """
        rospy.loginfo("Action Client Node running...")
        while not rospy.is_shutdown():
            cmd = input("Enter 's' to send a goal, 'c' to cancel, or 'q' to quit: ")
            if cmd == 's':
                x = float(input("Enter target x: "))
                y = float(input("Enter target y: "))
                self.send_goal(x, y)
            elif cmd == 'c':
                self.cancel_goal()
            elif cmd == 'q':
                break

if __name__ == "__main__":
    node = ActionClientNode()
    node.run()