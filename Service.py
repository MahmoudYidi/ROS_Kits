#### Define srv file as follows MathOperations.srv###################

int32 a
int32 b
---
int32 result
#####################################

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from your_package_name.srv import MathOperations, MathOperationsResponse

class MathOperationsServer:
    def __init__(self):
        self.addition_service = rospy.Service('addition_service', Trigger, self.handle_addition)
        self.subtraction_service = rospy.Service('subtraction_service', Trigger, self.handle_subtraction)

    def handle_addition(self, req):
        result = req  # Modify this to perform the addition operation
        addition_result = result.a + result.b  # Add the two integers
        response = TriggerResponse()
        response.success = True
        response.message = "Addition operation successful"
        return response

    def handle_subtraction(self, req):
        result = req  # Modify this to perform the subtraction operation
        response = TriggerResponse()
        response.success = True
        response.message = "Subtraction operation successful"
        return response

if __name__ == '__main__':
    rospy.init_node('math_operations_server')
    server = MathOperationsServer()
    rospy.spin()