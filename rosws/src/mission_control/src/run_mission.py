#!/usr/bin/env python
import rospy

from mission_controller import MissionController

if __name__ == '__main__':
    try:
        rospy.init_node('mission_runner', anonymous=False)        
        controller = MissionController(n_total_cubes=7, n_target_cubes=3, global_frame_id='map')
        controller.run()
    except rospy.ROSInitException:
        rospy.loginfo('Mission controller ended')