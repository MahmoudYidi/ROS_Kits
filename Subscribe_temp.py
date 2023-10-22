#!/usr/bin/env python

#Script 1
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received: %s", data.data)

def subscriber_node():
    # Initialize the ROS node
    rospy.init_node('basic_subscriber_node', anonymous=True)

    # Subscribe to the 'chatter' topic with String message type
    rospy.Subscriber('chatter', String, callback)

    # Spin to keep the node alive and continue receiving messages
    rospy.spin()

if __name__ == '__main__':
    subscriber_node()
    
# Script 2

######yaml#######
defined in publisher
#################

#!/usr/bin/env python

import rospy
from your_package_name.msg import MyCustomMessage

def callback(data):
    rospy.loginfo("Received: %s, timestamp: %s", data.value, data.timestamp)

def subscriber_node():
    # Initialize the ROS node
    rospy.init_node('custom_subscriber_node', anonymous=True)

    # Subscribe to the 'custom_data' topic with MyCustomMessage message type
    rospy.Subscriber('custom_data', MyCustomMessage, callback)

    # Spin to keep the node alive and continue receiving messages
    rospy.spin()

if __name__ == '__main__':
    subscriber_node()