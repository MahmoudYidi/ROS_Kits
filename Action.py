#### First Create the msg and action files ####
#AddNumbers.msg
int32[] numbers

#### AddNumbers.action
int32[] numbers
---
int32 sum

#####  Now you can create the Addition Server ########

#!/usr/bin/env python

import rospy
import actionlib
from your_package_name.msg import AddNumbersAction, AddNumbersGoal, AddNumbersResult

class AdditionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('add_numbers', AddNumbersAction, self.execute, auto_start=False)
        self.server.start()

    def execute(self, goal):
        result = AddNumbersResult()
        result.sum = sum(goal.numbers)

        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('addition_server')
    server = AdditionServer()
    rospy.spin()
    
##### Finally get the client in another script
#!/usr/bin/env python

import rospy
import actionlib
from your_package_name.msg import AddNumbersAction, AddNumbersGoal

def action_client():
    client = actionlib.SimpleActionClient('add_numbers', AddNumbersAction)
    client.wait_for_server()

    goal = AddNumbersGoal()
    goal.numbers = [1, 2, 3, 4, 5]  # Replace with your numbers

    client.send_goal(goal)
    client.wait_for_result() ####

    result = client.get_result()
    rospy.loginfo("Result: %d", result.sum)

if __name__ == '__main__':
    rospy.init_node('addition_client')
    action_client()