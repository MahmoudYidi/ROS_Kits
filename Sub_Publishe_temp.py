############################# About ###################################################
#In this script, we have two nodes. The first node (subscribe_and_publish) initializes the 
# ROS node, creates a subscriber that listens to the 'numbers' topic with Int32MultiArray message type, 
# and creates a publisher on the 'result' topic with Int32 message type. 
# When the subscriber receives a message, it calls the subscriber_callback function to perform
# the addition operation and publish the result.
###################################################################################################

#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

def subscriber_callback(data, pub):
    # Perform the operation (addition)
    result = sum(data.data)

    # Publish the result to the 'result' topic
    result_msg = Int32()
    result_msg.data = result
    pub.publish(result_msg)

def subscribe_and_publish():
    # Initialize the ROS node
    rospy.init_node('addition_node', anonymous=True)

    # Create a subscriber on the 'numbers' topic with Int32MultiArray message type
    rospy.Subscriber('numbers', Int32MultiArray, subscriber_callback, callback_args=pub)

    # Create a publisher on the 'result' topic with Int32 message type
    pub = rospy.Publisher('result', Int32, queue_size=10)

    # Spin to keep the node alive and continue receiving and publishing messages
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_and_publish()
    except rospy.ROSInterruptException:
        pass
#####################################################################################

import rospy
from std_msgs.msg import Int32

def callback(data, pub_add, pub_sub):
    # Perform addition operation
    result_add = data.data + 5  # Add 5 to the received data

    # Publish the addition result to the 'result_add' topic
    pub_add.publish(Int32(result_add))

    # Perform subtraction operation
    result_sub = data.data - 2  # Subtract 2 from the received data

    # Publish the subtraction result to the 'result_sub' topic
    pub_sub.publish(Int32(result_sub))

def subscribe_and_publish():
    # Initialize the ROS node
    rospy.init_node('math_operations_node', anonymous=True)

    # Create a subscriber on the 'input_data' topic with Int32 message type
    rospy.Subscriber('input_data', Int32, callback, callback_args=(pub_add, pub_sub))

    # Create publishers for addition and subtraction results
    pub_add = rospy.Publisher('result_add', Int32, queue_size=10)
    pub_sub = rospy.Publisher('result_sub', Int32, queue_size=10)

    # Spin to keep the node alive and continue receiving and publishing messages
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_and_publish()
    except rospy.ROSInterruptException:
        pass