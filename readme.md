# ROS1 Assignment: Robot Controller

## Description
This part implements a robot controller in a ROS1 environment using the simulation environment provided by Gazebo. The controller is divided into two key nodes:

1. **Action Client Node**: Sends goals to move the robot to specified positions and publishes the robot's real-time state (position and velocity).
2. **Service Node**: Responds to service requests by providing the robot's last goal target.

The robot performs a basic movement behavior with user-defined control.

---

## Nodes Overview

### 1. **Action Client Node**

#### Node Name: `action_client_node`

**Responsibilities:**
- Sends goals to an action server (`/reaching_goal`).
- Publishes robot state (position and velocity) to the topic `/robot_state`.
- Allows the user to input goals interactively via terminal.

**Published Topics:**
- `/robot_state` (Custom Message `RobotState`): Publishes the robot's position and velocity in real time.

**Subscribed Topics:**
- `/odom` (Message `nav_msgs/Odometry`): Subscribes to the robot's odometry data to track its position and velocity.

**Usage:**
```bash
rosrun assignment2_2024 action_client_node.py
```

### 2. **Service Node**

#### Node Name: `service_node`

**Responsibilities:**
- Provides a service `/get_target` to return the robot's last goal target.

**Service:**
- `/get_target` (Custom Service `GetTarget`): Returns the last target coordinates.

**Usage:**
```bash
rosrun assignment2_2024 service_node.py
```

---

## Launch File

**Launch File Name:** `assignment1.launch`

This launch file initializes the following:
- Simulation environment from Gazebo.
- Action client node.
- Service node.

**Note:** If you want to run the action client node in a separate terminal for better UI clarity, comment out its launch line in the file and uncomment the line launching it in a separate konsole window.

**Usage:**
```bash
roslaunch assignment_2_2024 assignment1.launch
```

To monitor the robot:
- **Check Robot State:**
  ```bash
  rostopic echo /robot_state
  ```
- **Check Robot Last Target:**
  ```bash
  rosservice call /get_target
  ```

---

## Custom Messages and Services

### 1. **Message: `RobotState`**
**File:** `msg/RobotState.msg`
```plaintext
float64 x
float64 y
float64 vel_x
float64 vel_z
```
**Purpose:** Publishes the robot's position (`x`, `y`) and velocity (`vel_x`, `vel_z`).

### 2. **Service: `GetTarget`**
**File:** `srv/GetTarget.srv`
```plaintext
---
float64 x
float64 y
```
**Purpose:** Returns the last target goal set for the robot.

---

## Usage Instructions

1. **Build the Workspace:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. **Run the Launch File:**
   ```bash
   roslaunch assignment2_2024 assignment1.launch

   ```

3. **Interact with the Nodes:**
   - Use the terminal running the action client node to send goals interactively.
   - Use the service call to query the last target:
     ```bash
     rosservice call /get_target

     ```

---

## Dependencies

Ensure the following dependencies are installed:
- ROS Noetic
- Simulation environment from Gazebo.
- `actionlib`
- `nav_msgs`
- `geometry_msgs`

---

## Author
Mazen Atta