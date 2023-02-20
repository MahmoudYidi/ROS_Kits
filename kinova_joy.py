#!/usr/bin/env python

'''
 This script assumes that you have a working Gazebo simulation of the Kinova Gen3 robot and have set up the appropriate ROS topics, publishers, and services to communicate with the simulation. You will need to modify the topic names in the script to match your particular setup.

The script subscribes to the /joy topic to receive joystick inputs and uses these inputs to generate commands for the robot. The left stick of the joystick controls the robot's linear motion in the X, Y, and Z directions, while the right stick controls the robot's angular motion in the roll, pitch, and yaw directions. The gripper is opened and closed using buttons 2 and 3 on the joystick, respectively.

The reset_simulation() function is called when button 1 is pressed on the joystick, and this function uses a ROS service to reset the Gazebo simulation.

Author: Mahmoud Abdulsalam
Email: mahmoudyidi@gmail.com
'''
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

# Constants
JOINT_SPEED = 0.1  # Joint speed in radians per second
CARTESIAN_SPEED = 0.01  # Cartesian speed in meters per second

# Global variables
joy_axes = [0.0] * 6
joy_buttons = [0] * 12

# Initialize ROS node
rospy.init_node('joystick_control')

# Create ROS publishers
joint_cmd_pub = rospy.Publisher('/joint_group_position_controller/command', Float64MultiArray, queue_size=1)
gripper_cmd_pub = rospy.Publisher('/gripper_controller/command', Float64, queue_size=1)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Create ROS services
reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

# Joystick callback function
def joystick_callback(msg):
    global joy_axes, joy_buttons
    joy_axes = msg.axes
    joy_buttons = msg.buttons

# Main function
def main():
    # Subscribe to joystick topic
    rospy.Subscriber('joy', Joy, joystick_callback)

    # Set loop rate
    rate = rospy.Rate(10)

    # Main loop
    while not rospy.is_shutdown():
        # Check for button presses
        if joy_buttons[0]:  # Button 1: reset simulation
            reset_simulation()

        # Check for joystick movements
        if abs(joy_axes[0]) > 0.1:  # Left stick X-axis: move in X direction
            cmd_vel_pub.publish(Twist(linear=Vector3(x=CARTESIAN_SPEED*joy_axes[0])))
        if abs(joy_axes[1]) > 0.1:  # Left stick Y-axis: move in Y direction
            cmd_vel_pub.publish(Twist(linear=Vector3(y=CARTESIAN_SPEED*joy_axes[1])))
        if abs(joy_axes[3]) > 0.1:  # Right stick Y-axis: move in Z direction
            cmd_vel_pub.publish(Twist(linear=Vector3(z=CARTESIAN_SPEED*joy_axes[3])))
        if abs(joy_axes[4]) > 0.1:  # Right stick X-axis: move in roll and pitch directions
            cmd_vel_pub.publish(Twist(angular=Vector3(x=JOINT_SPEED*joy_axes[4], y=JOINT_SPEED*joy_axes[4])))
        if abs(joy_axes[5]) > 0.1:  # D-pad X-axis: move in yaw direction
            cmd_vel_pub.publish(Twist(angular=Vector3(z=JOINT_SPEED*joy_axes[5])))

        # Check for gripper commands
        if joy_buttons[1]:  # Button 2: open gripper
            gripper_cmd_pub.publish(0.08)
        elif joy_buttons[2]:  # Button 3: close gripper
            gripper_cmd_pub.publish(-0.08)

        # Sleep for the remainder of the loop
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
