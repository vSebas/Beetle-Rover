#!/usr/bin/python
# This script needs to be run like this: 
# python -m src.node_tests.controller_test
import rospy
import unittest
from ..controller import Controller

from locomotion_control_node.srv import set_controller_mode, set_controller_modeResponse, set_controller_modeRequest
from locomotion_control_node.srv import stop, stopRequest, stopResponse
class TestControllerNode(unittest.TestCase):
    

    def test_controller(self):
        controller = Controller(motors_connected={"left":[True, True] , "right": []})
        self.stop_proxy = rospy.ServiceProxy('/mtr_ctr_0_stop_motors', stop)
        print self.stop_proxy(True)
        # self.control_mode = rospy.ServiceProxy('/mtr_ctr_0_mode_service', set_controller_mode)
        # self.assertTrue('Successfully' in str(self.control_mode('PWM')) and controller.get_controller_mode()=='PWM')
        # self.assertTrue('Successfully' in str(self.control_mode('SPEED'))and controller.get_controller_mode()=='SPEED')
        # left_list = []
        # for motor in controller.get_motors_connected("left"):
        #     left_list.append(motor.get_name())
        # right_list = []
        # for motor in controller.get_motors_connected("right"):
        #     right_list.append(motor.get_name())
        # self.assertIn("M1", left_list) 
        # self.assertIn("M2", left_list)
        # self.assertNotIn( "M1", right_list)
        # self.assertNotIn( "M2", right_list)
        

if __name__ == '__main__':

    rospy.init_node('controller_test')

    unittest.main()
