#!/usr/bin/python
from roboclaw import Roboclaw
import rospy
from locomotion_control_node.srv import set_controller_mode, set_controller_modeRequest, set_controller_modeResponse
from locomotion_control_node.msg import encoder_values, speed_values
from rover_config_container import RoverConfig
from locomotion_control_node.srv import stop, stopRequest, stopResponse
import numpy

class Motor (object):
    def __init__(self, side=None, name=None):
        self.side = side
        self.name = name
        self.speed = 0
        print self.name

    def get_speed (self):
        return self.speed

    def set_speed(self, speed):
        self.speed = speed

    def get_side (self):
        return self.side
    def get_name (self):
        return self.name
    def isM1(self):
        if self.name == "M1":
            return True
        return False
    def isM2(self):
        if self.name == "M2":
            return True
        return False

class Controller(object):
    def __init__(self, serial_port = '/dev/ttyACM0', baudrate = 115200, mode='PWM', motors_connected = {"left":[], "right": []}, index=0):
        self.name = "mtr_"+str(index)
        self._serial_port = serial_port
        self._baudrate = baudrate
        self._address = 0x80
        self._rc = Roboclaw(self._serial_port, self._baudrate)
        self._rc.Open()
        self.stop_service = rospy.Service(self.name+'_stop_motors', stop, self.stop_srv)
        self._mode = mode
        self.current_speed=0
        if len(motors_connected["left"])+len(motors_connected["right"]) > 2:
            raise ValueError("You can only connect two motors to these controllers!")
        self.motors_connected = {"left": [], "right": []}
        self.motor_number = 0
        for side in motors_connected.keys():
            for motor in motors_connected[side]:
                if motor:
                    if self.motor_number == 0:
                        self.motors_connected[side].append(Motor(name=str(motor), side=side))
                        self.motor_number += 1
                    elif self.motor_number == 1:
                        self.motors_connected[side].append(Motor(name=str(motor), side=side))
                    else:
                        raise ValueError('Too many motors, numer is: '+ str(self.motor_number))
        

        ## ROS Interfaces:
        self.r = rospy.Rate(10)
        self.encoder_publisher = rospy.Publisher(self.name+'_enc_val', encoder_values, queue_size=1)
        self.speed_publisher = rospy.Publisher(self.name+'_speed_val', speed_values, queue_size=1)
        self.mode_service = rospy.Service(self.name+"_mode_service", set_controller_mode, self.set_controller_mode)

    def get_motors_connected(self, side=None):
        if side == None or side not in ["left", "right"]:
            return self.motors_connected
        else:
            return self.motors_connected[side]


    def get_controller_mode(self):
        return self._mode

    def set_controller_mode(self, data):
        if data.controller_mode not in ['SPEED', 'PWM']:
            rospy.logerr_once( 'ERROR: Given controller mode is not supported, either enter "SPEED" or "PWM"'+ ', I got: '  +str(data.controller_mode))
            return set_controller_modeResponse( "Cannot change mode to: "  +str(data.controller_mode))
        self._mode = data.controller_mode
        return set_controller_modeResponse('Successfully changed the controller mode to {0}'.format(self._mode))

    ## Mapping of RC Control methods:
    def set_speed(self, side, speed):
        #print "I got: "+ str(side)+ " and " + str(speed)
        # if speed == self.current_speed:
        #     return
        if self.motors_connected[side] == []:
            return
        for motor in self.motors_connected[side]:
            if motor.speed == speed:
                return
            acceleration = 2000
            if abs(speed) > 10000 and self._mode=="SPEED":
                speed = 10000*numpy.sign(speed)
            if abs(speed) > 16 and self._mode=="PWM":
                speed = 16*numpy.sign(speed)
            print "speed1:" +str(speed)
            if motor.isM1() == True:
                if self._mode == "SPEED":
                    print "Sending to M1:" + str(speed)
                    try:
                        self._rc.SpeedAccelM1(self._address, acceleration, speed)
                    except:
                        rospy.loginfo("Cannot communicate with controllers")
                elif self._mode == "PWM":
                    pwm = 64 + speed
                    print "sending to M1: " +str(pwm)
                    try:
                        self._rc.ForwardBackwardM1(self._address, pwm)
                    except:
                        rospy.loginfo("Cannot communicate with controller")
            print "speed2:" +str(speed)
            if motor.isM2() == True:
                if self._mode == "SPEED":
                    print "Sending to M2:" + str(speed)
                    try:
                        self._rc.SpeedAccelM2(self._address, acceleration, speed)
                    except:
                        rospy.loginfo("Cannot communicate with controllers")
                elif self._mode == "PWM":
                    pwm = 64 + speed
                    print "sending to M2: " +str(pwm)
                    try:
                        self._rc.ForwardBackwardM2(self._address, pwm)
                    except:
                        rospy.loginfo("Cannot communicate with controller")
            
            motor.set_speed(speed)

    ## Stop Method
    def stop(self):
        rospy.loginfo(self.name+': Stopping both motors')
        if self._mode == "SPEED":
            try:
                self._rc.SpeedM1M2(self._address, 0, 0)
            except:
                rospy.logerr('Could not contact controller to stop motor!!!')
        if self._mode == "PWM":
            try:
                self._rc.ForwardBackwardM1(self._address, 64)
                self._rc.ForwardBackwardM2(self._address, 64)
            except:
                rospy.logerr('Could not contact controller to stop motor!!!')
        return

    ## Stop service method
    def stop_srv(self, data):
        if data.request:
            self.stop()
            return stopResponse('Motors are stopped')
        return stopResponse("Don't call stop service with false, who does that?") 







    def run_publishers(self):
        rc = self._rc
        address = self._address
        encoder_message = encoder_values()
        speed_message = speed_values()

        enc1 = rc.ReadEncM1(address)
        enc2 = rc.ReadEncM2(address)   
        speed1 = rc.ReadSpeedM1(address)
        speed2 = rc.ReadSpeedM2(address)
        if(enc1[0]==1):
            encoder_message.enc1 = enc1[1]
        else:
            rospy.logerr(self.name+': Could not read data from first motor')  

        if(enc2[0]==1):
            encoder_message.enc2 = enc2[1]
        else:
            rospy.logerr(self.name+': Could not read data from second motor')
        
        if(speed1[0]):
            print speed1[1],
        else:
            rospy.logerr(self.name+': Could not read data from encoder 2')
        print("Speed2:"),
        if(speed2[0]):
            print speed2[1]
        else:
            print "failed "    
            


   

    


if __name__ == "__main__":

    


    rospy.init_node('Controller_Node_Test')
    controller = Controller()
    
    
    
    #controller._rc.SpeedAccelM1(controller._address, acc, vel)
    while not rospy.is_shutdown():
        #controller.run_publishers()
        controller.r.sleep()