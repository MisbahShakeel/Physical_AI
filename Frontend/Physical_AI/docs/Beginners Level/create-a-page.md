---
sidebar_position: 1
---

# Getting Started with Physical AI Simulation

Learn how to create and run your first **Physical AI and Robotics simulation**:

- Set up your development environment with ROS and Gazebo
- Create your first robot model and simulation world
- Implement basic AI behaviors for robot control

## Create your first Robot Simulation

Create a basic robot model file at `robots/my_first_robot.urdf`:

```xml title="robots/my_first_robot.urdf"
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

Load your robot into Gazebo simulation to test its basic properties.

## Create your first AI Behavior Script

Create a Python script at `scripts/move_robot.py` to implement basic robot movement:

```python title="scripts/move_robot.py"
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_robot():
    rospy.init_node('robot_mover', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Set linear velocity
    vel_msg.linear.x = 1.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0

    # Set angular velocity
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.5

    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
```

This script will make your robot move in a circular pattern in the simulation.
