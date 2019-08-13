#!/usr/bin/env python

from mission_controller import MissionController

if __name__ == '__main__':
    try:
        rospy.init_node('mission_runner', anonymous=False)        
        controller = MissionController(n_total_cubes=7, n_target_cubes=3)
        controller.run()
    except rospy.ROSInitException:
        rospy.loginfo('Mission ended')