# Obstacle Avoiding Robot ROS
The purpose of this package is to provide a central decision-making node for controlling an 
obstacle avoiding robot equipped with an ultrasonic sensor.

## Nodes
**robot_control:** Provides the decision making protocols for the robot.

## Launch Files
**robot_control.launch:** Launches the robot_control node as well as the nodes required for 
the sensors and motor controller. Set the args sensor_port and motor_port to their respective
values as required.
