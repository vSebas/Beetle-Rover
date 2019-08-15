import rospy
import smach
import smach_ros

from mission_controller import MissionController

class Initialization(smach.State):
    def __init__(self, mission_controller):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self._mission_controller = mission_controller
    
    def execute(self, userdata):
        rospy.loginfo('Executing the INITIAL_CHECK state')
        self._mission_controller.initialization()
        return 'success'

class YawRotationScan(smach.State):
    def __init__(self, mission_controller):
        smach.State.__init__(self, outcomes=['finished'])
        self._mission_controller = mission_controller
    
    def execute(self, userdata):
        rospy.loginfo('Executing the YAW_ROTATION_SCAN state')
        self._mission_controller.yaw_rotation_scan()
        return 'finished'

class Explore(smach.State):
    def __init__(self, mission_controller):
        smach.State.__init__(self, outcomes=['no_visits_pending', 'all_cubes_visited', 'visits_pending'])
        self._mission_controller = mission_controller
    
    def execute(self, userdata):
        rospy.loginfo('Executing the EXPLORE state')
        self._mission_controller.explore()
        if self._mission_controller.all_target_cubes_visited():
            return 'all_cubes_visited'
        else:
            if self._mission_controller.target_cubes_pending_visit():
                return 'visits_pending'
            else:
                return 'no_visits_pending'

class VisitGoodCube(smach.State):
    def __init__(self, mission_controller):
        smach.State.__init__(self, outcomes=['explore', 'all_cubes_visited', 'visit_next'])
        self._mission_controller = mission_controller
    
    def execute(self, userdata):
        rospy.loginfo('Executing the VISIT_GOOD_CUBE state')
        if self._mission_controller.visit_next_target_cube():
            # visit was successful
            if self._mission_controller.all_target_cubes_visited():
                return 'all_cubes_visited'
            elif self._mission_controller.target_cubes_pending_visit():
                return 'visit_next'
        else:
            # visit was unsuccessful
            return 'explore'


        return 'finished'

class Finish(smach.State):
    def __init__(self, mission_controller):
        smach.State.__init__(self, outcomes=['success'])
        self._mission_controller = mission_controller
    
    def execute(self, userdata):
        rospy.loginfo('Executing the FINISH state')
        self._mission_controller.finalize()
        return 'success'

def create_mission_state_machine(**kwargs):
    sm = smach.StateMachine(outcomes=['success', 'failure'])
    mission_controller = MissionController(**kwargs)
    with sm:
        initialization_transitions = {
            'success': 'YAW_ROTATION_SCAN',
            'failure': 'FINISH'
        }
        sm.add('INITIALIZATION', Initialization(mission_controller), initialization_transitions)
       
        yaw_rotation_scan_transitions = {
            'finished': 'VISIT_GOOD_CUBE'
        }
        sm.add('YAW_ROTATION_SCAN', YawRotationScan(mission_controller), yaw_rotation_scan_transitions)
       
        explore_transitions = {
            'no_visits_pending': 'YAW_ROTATION_SCAN',
            'all_cubes_visited': 'FINISH',
            'visits_pending': 'VISIT_GOOD_CUBE'
        }
        sm.add('EXPLORE', Explore(mission_controller), explore_transitions)

        visit_good_cube_transitions = {
            'explore': 'EXPLORE',
            'visit_next': 'VISIT_GOOD_CUBE',
            'all_cubes_visited': 'FINISH'
        }
        sm.add('VISIT_GOOD_CUBE', VisitGoodCube(mission_controller), visit_good_cube_transitions)

        sm.add('FINISH', Finish(mission_controller))

    return sm