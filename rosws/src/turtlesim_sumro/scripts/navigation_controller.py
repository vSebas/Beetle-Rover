#!/usr/bin/env python

"""
The navigation controller commands the robot movement so that it visits each target cube once.

References:
1) Motive: How to set navigation goals?
   Source:
      * https://gitlab.tubit.tu-berlin.de/l.kryza/sumro/blob/master/ros_reference/rosws/src/turlesim_sumro/src/move_base_square.py

2) Motive: How to use a transformation listener?
   Source:
      * https://gitlab.tubit.tu-berlin.de/l.kryza/sumro/blob/master/ros_reference/rosws/src/turlesim_sumro/src/tflookup.py
      * http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29

3) Motive: Debug an error when using the transform listener
   Source:
      * https://answers.ros.org/question/203274/frame-passed-to-lookuptransform-does-not-exist/?answer=203281#post-id-203281
"""

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Transform, Twist, Quaternion

import tf2_ros

class Cube(object):
    """ Represents a cube in the map """
    
    def __init__(self, number):
        self._visited = False
        self._number = number
        self._xpos = None
        self._ypos = None
    
    @property
    def number(self):
        return self._number

    @property
    def visit_pending(self):
        """ Returns True if a visit to this cube is pending """
        return self.is_target and not self.visited
    
    @property
    def is_target(self):
        """ Returns True if this cube should be visited """
        return self.number % 2 != 0
    
    @property
    def visited(self):
        """ Returns True if this cube is marked as visited """
        return self._visited

    @property
    def xpos(self):
        return self._xpos
    
    @property
    def ypos(self):
        return self._ypos
    
    @xpos.setter
    def set_xpos(self, xpos):
        self._xpos = xpos
    
    @ypos.setter
    def set_ypos(self, ypos):
        self._ypos = ypos
    
    def visit(self):
        """ Marks cube as visited """
        self._visited = True

    def __str__(self):
        return 'Cube (number={}, visited={})'.format(self.number, self.visited)

class NavigationController(object):
    """
    The navigator controller makes the robot 
    visit each good cube and avoids the bad cubes.
    """

    def __init__(self):
        rospy.on_shutdown(self._shutdown)

        # We know there will be 7 cubes in total for the task
        self._cubes = [Cube(i) for i in range(1, 8)]

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._cmd_vel_pub = rospy.Publisher('base_footprint/cmd_vel', Twist, queue_size=5)

    def get_cube_transformation(self, cube):
        """
        Returns the transform from the cube's coordinate frame
        to the world's coordinate frame
        """
        return self._get_frame_transform('world', 'Cube{}'.format(cube.number))

    def move_to_cube(self, cube):
        """
        Creates a goal to move towards a cube
        and sends that goal to the move base server
        """
        if cube.visit_pending:
            rospy.loginfo('Attempting to move towards {}'.format(cube))
            self._move_base_client.wait_for_server(rospy.Duration(4.0))

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'world'
            goal.target_pose.header.stamp = rospy.Time.now()
            trans = self.get_cube_transformation(cube)

            goal.target_pose.pose.position.x = trans.transform.translation.x
            goal.target_pose.pose.position.y = trans.transform.translation.y
            goal.target_pose.pose.position.z = 0
            goal.target_pose.pose.orientation = trans.transform.rotation

            if self._move(goal):
                cube.visit()
            else:
                rospy.loginfo('Failed to move towards {}'.format(cube))
        else:
            rospy.loginfo('Skipping {}'.format(cube))

    def run(self):
        pending_cubes = self._pending_cubes()
        while not rospy.is_shutdown() and pending_cubes:
            self.move_to_cube(pending_cubes[0])
            pending_cubes = self._pending_cubes()

    def _pending_cubes(self):
        """ 
        Returns a list of the discovered (good) cubes
        TODO: Sort the list by distance to the robot
        """
        visit_pending = lambda cube: cube.visit_pending
        return filter(visit_pending, self._cubes)

    def _move(self, goal):
        """ 
        Adapted from source 1)
        Returns True is the move was successful
        """
        # Did we reach our goal?
        success = False

        # Send the goal pose to the MoveBaseAction server
        self._move_base_client.send_goal(goal)
        
        # Allow 1 minute to get there
        # TODO: adjust duration for a real-case scenario
        finished_within_time = self._move_base_client.wait_for_result(rospy.Duration(60)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self._move_base_client.cancel_goal()
            rospy.loginfo('Timed out achieving goal')
        else:
            # We made it!
            state = self._move_base_client.get_state()
            success = state == GoalStatus.SUCCEEDED
            rospy.loginfo('Goal state: {}'.format(state))
        return success

    def _get_frame_transform(self, target_frame, source_frame, max_retries=None):
        """
        Returns the transform from source_frame's coordinate frame
        to the target_frame's coordinate frame
        """
        trans = None
        current_retries = 0
        while trans is None and (max_retries is None or current_retries < max_retries):
            try:
                trans = self._tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time.now(), rospy.Duration(4.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr('Error getting frame transform ({} -> {}): {}'.format(source_frame, target_frame, e))
                current_retries += 1
        return trans

    def _shutdown(self):
        """ Taken from source 1) """
        rospy.loginfo('Stopping the robot...')
        # Cancel any active goals
        self._move_base_client.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self._cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':

    try:
        rospy.init_node('navigation_controller', anonymous=False)        
        controller = NavigationController()
        controller.run()
    except rospy.ROSInitException:
        rospy.loginfo('Navigation Controller ended.')