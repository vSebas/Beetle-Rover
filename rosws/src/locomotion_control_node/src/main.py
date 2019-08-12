#!/usr/bin/python

import rospy
import roboclaw
from geometry_msgs.msg import Twist
import math 
import numpy 
from rover_config_container import RoverConfig
from controller import Controller
from locomotion_control_node.srv import stop, stopResponse


class LocmotionControlNode(object):
    def __init__(self):
        self.name = 'sumro_lc_node'
        rospy.init_node(self.name)
        self.cmd_vel_subcriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rover_config = RoverConfig('rover_config.yaml')
        self.rover_speed_states = {'linear_x': 0.0, 'left': 0.0, 'right': 0.0, 'angular_z': 0.0}
        self.control_mode = rover_config.get_controller_mode()
        self.wheel_radius = rover_config.get_wheel_radius()
        # Factor for converting m/s in rounds/s
        self.rps_factor = 1/(2*math.pi*self.wheel_radius)
        self.wheel_distance = rover_config.get_wheel_distance()
        self.qpps_factor = rover_config.get_qpps_factor()
        self.pwm_factor = rover_config.get_pwm_factor()
        self.controllers = {}
        controllers = rover_config.get_controllers()
        for controller in controllers.keys():
            sp = controllers[controller]['serial_port']
            bd = controllers[controller]['baudrate']
            mocon = {'left': [], 'right': []}
            for side in ['left', 'right']:
                if controllers[controller][side] is None:
                    continue
                for motor in controllers[controller][side]:
                    mocon[side].append(motor)
            self.controllers[controller] = Controller(serial_port=sp, baudrate=bd, mode=self.control_mode, motors_connected=mocon, index=controller)
        print self.controllers

        self._left_side_velocity = None
        self._right_side_velocity = None
        self._velocity = [self._left_side_velocity, self._right_side_velocity]
        self.handbreaks = False
        # Adding handbreak service
        self.hb_srv = rospy.Service(self.name+'_handbreaks', stop, self.handbreaks_handler)
        
    
    def handbreaks_handler(self, data):
        if data.request:
            self.handbreaks = True
            for controller in self.controllers.keys():
                self.controllers[controller].stop()
                return stopResponse('I initated full stop!')
        if data.request == False:
            self.handbreaks = False
            return stopResponse('Handbreaks are released.')


        
        


    def cmd_vel_callback(self, data):
        
        if self.handbreaks:
            rospy.loginfo ('Getting velocity commands, but handbreaks are on!')
            return
        lx = data.linear.x
        if abs(lx) > 0.3:
            lx = 0.3 * numpy.sign(lx)
        rz = data.angular.z
        # Correction for rover behaviour:
        if abs(rz) < 0.2:
            rz = 0.2 * numpy.sign(rz)
        if abs(rz) > 1.0:
            rz = 1.0*numpy.sign(rz)
        rz = rz * 1.44 # <- this factor may vary
        lr = rz*self.wheel_distance
        if True:#lx >= 0:
            v_l = lx - lr
            v_r = lx + lr
        else:
            v_l = lx + lr
            v_r = lx - lr
        
        # converting rps into qpps commands for motors:
        if self.control_mode == "SPEED":
            # Converting velocties in rounds per second:
            [rps_l, rps_r] = [x*self.rps_factor for x in [v_l, v_r]]
            [qpps_l, qpps_r] = [int(x*self.qpps_factor) for x in [rps_l, rps_r]]
            for controller in self.controllers.keys():
                self.controllers[controller].set_speed('left', qpps_l)
                self.controllers[controller].set_speed('right', qpps_r)
        elif self.control_mode == "PWM":
            [pwm_l, pwm_r] = [int(x*self.pwm_factor) for x in [v_l, v_r]]
            for controller in self.controllers.keys():
                self.controllers[controller].set_speed('left', pwm_l)
                self.controllers[controller].set_speed('right', pwm_r)
            print "I am getting: " + str([pwm_l, pwm_r])
        



if __name__ == "__main__":
    slc = LocmotionControlNode()
    rospy.spin()
