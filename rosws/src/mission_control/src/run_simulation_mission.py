#!/usr/bin/env python
import rospy

#from mission_controller import MissionController
from mission_sm import create_mission_state_machine

if __name__ == '__main__':
    try:
        rospy.init_node('simulation_mission_runner', anonymous=False)        
        #controller = MissionController(n_total_cubes=7, n_target_cubes=3)
        #controller.yaw_rotation_scan()
        #controller.run()
        sm = create_mission_state_machine(n_total_cubes=7, n_target_cubes=4)
        outcome = sm.execute()
    except rospy.ROSInitException:
        rospy.loginfo('Mission controller ended')