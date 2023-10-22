#!/usr/bin/env python

# Script 1
import rospy
from std_msgs.msg import String

def publisher_node():
    # Initialize the ROS node
    rospy.init_node('basic_publisher_node', anonymous=True)

    # Create a publisher on the 'chatter' topic with String message type
    pub = rospy.Publisher('chatter', String, queue_size=10)

    # Rate at which to publish (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Create a String message
        message = String()
        message.data = "Hello, ROS!"

        # Publish the message
        pub.publish(message)

        # Log the published message
        rospy.loginfo("Published: %s", message.data)

        # Sleep to control the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
    
# Script 2

#####  Message yaml file ######
string value
time timestamp
###############################

#!/usr/bin/env python

import rospy
from your_package_name.msg import MyCustomMessage
from std_msgs.msg import Header
import time

def publisher_node():
    # Initialize the ROS node
    rospy.init_node('custom_publisher_node', anonymous=True)

    # Create a publisher on the 'custom_data' topic with MyCustomMessage message type
    pub = rospy.Publisher('custom_data', MyCustomMessage, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Create a MyCustomMessage instance
        custom_msg = MyCustomMessage()
        custom_msg.value = "Hello, ROS!"
        custom_msg.timestamp = rospy.Time.now()

        # Publish the message
        pub.publish(custom_msg)

        # Log the published message
        rospy.loginfo("Published: %s, timestamp: %s", custom_msg.value, custom_msg.timestamp)

        # Sleep to control the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass