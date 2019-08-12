#!/usr/bin/python

import rospy

from yaml_handler import YamlHandler
class RoverConfig(YamlHandler):
    def __init__(self, config_file='rover_config.yaml',*args, **kwargs):
        super(RoverConfig, self).__init__(config_file, args, kwargs)
        self.read_config(self.get_file())
        
         
    def read_config(self, config):
        self._wheel_radius = config['wheel_radius']
        self._distance_wheels = config['distance_center_wheels']
        self._speed_qpps_factor = config['speed_qpps_factor']
        self._pwm_factor = config['pwm_factor']
        self._controller_mode = config['controller_mode']
        self._controllers = config['controllers']

    def get_controllers(self):
        return self._controllers 

    def get_wheel_radius(self):
        return self._wheel_radius
    def get_wheel_distance(self):
        return self._distance_wheels
    def get_qpps_factor(self):
        return self._speed_qpps_factor
    def get_pwm_factor(self):
        return self._pwm_factor
    def get_controller_mode(self):
        return self._controller_mode
    def get_qpps_factor(self):
        return self._speed_qpps_factor
    




if __name__ == '__main__':
    import time 
    #rospy.init_node('test')
    #time.sleep(1)
    rover_config = RoverConfig('rover_config.yaml')
    print rover_config.get_controllers().keys()
