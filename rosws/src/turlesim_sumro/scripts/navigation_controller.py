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
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

class Cube(object):
    """
    Represents a cube in the map
    """
    
    def __init__(self, number):
        self._number = number
    
    @property
    def frame_id(self):
        return 'Cube{}'.format(self._number)
    
    @property
    def is_good(self):
        # rule: odd numbers are good cubes
        return self._number % 2 != 0

    def __str__(self):
        return 'Cube #{}'.format(self._number)

class NavigationController(object):
    """
    The navigator controller makes the robot 
    visit each good cube and avoids the bad cubes.
    """

    def __init__(self, cubes=[]):
        rospy.init_node('navigation_controller', anonymous=False)        
        rospy.on_shutdown(self._shutdown)

        self._cubes = cubes
        self._transform_listener = tf.TransformListener()
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('base_footprint/cmd_vel', Twist, queue_size=5)

    def get_cube_pose(self, cube):
        try:
            self._transform_listener.waitForTransform('world', cube.frame_id, rospy.Time(), rospy.Duration(4.0))
            (position, orientation) = self._transform_listener.lookupTransform('world', cube.frame_id, rospy.Time(0))
            return position, orientation
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('Error getting position and orientation: {}'.format(e))
        return None, None

    def move_to_cube(self, cube):
        """ Creates a goal to move towards a cube
            and sends that goal to the move base server
        """
        if cube.is_good:
            rospy.loginfo('Moving towards good cube: {}'.format(cube))
            self._move_base_client.wait_for_server(rospy.Duration(4.0))
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'world'
            goal.target_pose.header.stamp = rospy.Time.now()
            position, orientation = self.get_cube_pose(cube)
            goal.target_pose.pose = Pose(Point(*position), Quaternion(*orientation))
            self._move(goal)
        else:
            rospy.loginfo('Skipping bad cube: {}'.format(cube))

    def run(self):
        i = 0
        while not rospy.is_shutdown() and i < len(self._cubes):
            self.move_to_cube(self._cubes[i])
            i += 1

    def _move(self, goal):
        """ 
        Adapted from source 1)
        """
        # Send the goal pose to the MoveBaseAction server
        self._move_base_client.send_goal(goal)
        
        # Allow 1 minute to get there
        finished_within_time = self._move_base_client.wait_for_result(rospy.Duration(60)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self._move_base_client.cancel_goal()
            rospy.loginfo('Timed out achieving goal')
        else:
            # We made it!
            state = self._move_base_client.get_state()
            rospy.loginfo('Goal state: {}'.format(state))

    def _shutdown(self):
        """
        Taken from source 1)
        """
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self._move_base_client.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    
    # Initialize 7 cubes
    cubes = [Cube(i) for i in range(1, 8)]

    # Initialize controller
    controller = NavigationController(cubes)
    controller.run()
