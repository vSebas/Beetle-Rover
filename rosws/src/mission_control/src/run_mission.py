#!/usr/bin/env python
import rospy

from mission_sm import create_mission_state_machine

if __name__ == '__main__':
    try:
        rospy.init_node('mission_runner', anonymous=False)
        sm = create_mission_state_machine(n_total_cubes=7, n_target_cubes=4, global_frame_id='map')
        outcome = sm.execute()
    except rospy.ROSInitException:
        rospy.loginfo('Mission controller ended')