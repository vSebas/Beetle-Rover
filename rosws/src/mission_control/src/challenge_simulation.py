#!/usr/bin/env python

"""
A node that simulates our rover's challenge.

Motivation: have different scenarios to test our code.


References:

1) https://wiki.ros.org/turtlesim

2) http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

3) https://github.com/ROS4SPACE/aruco_analyzer/blob/e50ab9183b8903ba5699115788f7f813f713aa6f/aruco_analyzer/src/aruco_analyzer/ros_wrapper/tf_broadcaster.py

4) http://docs.ros.org/melodic/api/turtle_tf/html/turtle__tf__broadcaster_8py_source.html
"""

import math
import random
import rospy
import tf2_ros
import turtlesim

from cube import Cube
from geometry_msgs.msg import TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler
from turtlesim.srv import *
import turtlesim.msg

class ChallengeSimulation(object):

    MIN_X = 1.5
    MAX_X = 9.0

    MIN_Y = 1.5
    MAX_Y = 9.0

    INFLUENCE_AREA_RADIUS = 5.5

    # Use a different orientation for evil and good cubes to tell them apart visually.
    EVIL_CUBE_ORIENTATION = math.radians(-90)
    GOOD_CUBE_ORIENTATION = math.radians(90)

    def __init__(self):
        rospy.on_shutdown(self._shutdown)
        # We know there will be 7 cubes in total for the task
        self._cubes = [Cube(i) for i in range(1, 8)]

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._base_footprint_pos_subscriber = None
        self._timer = None
        self._global_frame_id = None
        
    def _initialize_coordinates(self):
        # Create coordinates set and add first point
        coordinates = []
        coordinates.append((self._uniform_dist(self.MIN_X, self.MAX_X), self._uniform_dist(self.MIN_Y, self.MAX_Y)))
        # Create the remaining points
        N = len(self._cubes)
        while(len(coordinates) != N):
            point = None
            while point is None:
                # our new candidate point
                x = self._uniform_dist(self.MIN_X, self.MAX_X)
                y = self._uniform_dist(self.MIN_Y, self.MAX_Y)
                for (x2, y2) in coordinates:
                    if self._is_inside_cube_area(x,y,x2,y2):
                        rospy.loginfo('Collision between turtles detected!')
                        break
                point = (x,y)
                coordinates.append(point)

        assert(len(coordinates) == 7)
        
        for cube, point in zip(self._cubes, coordinates):
            cube.xpos = point[0]
            cube.ypos = point[1]

    def _is_inside_cube_area(self, x1, y1, x2, y2):
        ''' Returns True if cube2's position is inside cube1's area of influece '''
        xcheck = x1 - self.INFLUENCE_AREA_RADIUS <= x2 and x2 <= x1 + self.INFLUENCE_AREA_RADIUS
        ycheck = y1 - self.INFLUENCE_AREA_RADIUS <= y2 and y2 <= y1 + self.INFLUENCE_AREA_RADIUS
        return xcheck and ycheck

    def _uniform_dist(self, a=0.0, b=1.0):
        sample = a + (b-a)*random.random()
        assert(sample < b)
        assert(sample >= a)
        return sample

    def _create_turtles(self):
        rospy.wait_for_service('spawn')
        for cube in self._cubes:
            try:
                spawn = rospy.ServiceProxy('spawn', Spawn)
                yaw_orientation = self.GOOD_CUBE_ORIENTATION if cube.is_target else self.EVIL_CUBE_ORIENTATION
                cube.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, yaw_orientation))
                response = spawn(cube.xpos, cube.ypos, yaw_orientation, cube.frame_id)
            except rospy.ServiceException, e:
                rospy.loginfo('Failed to spawn turtle. ' + str(e))

    def _create_rover(self):
        rospy.wait_for_service('spawn')
        try:
            spawn = rospy.ServiceProxy('spawn', Spawn)
            yaw_angle = math.radians(self._uniform_dist(0.0, 360.0))
            response = spawn(0.5, 0.5, yaw_angle, 'base_footprint')
        except rospy.ServiceException, e:
            rospy.loginfo('Failed to spawn turtle. ' + str(e))
        else:
            self._base_footprint_pos_subscriber = rospy.Subscriber('/base_footprint/pose', turtlesim.msg.Pose, callback=self._broadcast_rover_tf_callback)

    def _broadcast_rover_tf_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self._global_frame_id
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(*quaternion_from_euler(0.0, 0.0, msg.theta))
  
        self._tf_broadcaster.sendTransform(t)

    def _broadcast_tfs_callback(self, event):
        for cube in self._cubes:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self._global_frame_id
            t.child_frame_id = cube.frame_id

            t.transform.translation.x = cube.xpos
            t.transform.translation.y = cube.ypos
            t.transform.translation.z = 0.0
            t.transform.rotation = cube.orientation
            
            self._tf_broadcaster.sendTransform(t)

    def run(self):
        self._global_frame_id = rospy.get_param('global_frame_id', default='world')
        self._initialize_coordinates()
        self._create_turtles()
        self._create_rover()
        self._timer = rospy.Timer(rospy.Duration(1.0/2.0), self._broadcast_tfs_callback) # 2 hz

    def _shutdown(self):
        if self._timer is not None:
            self._timer.shutdown()

def main():
    try:
        random.seed()
        rospy.init_node('sumro_challenge_simulation', anonymous=False)
        challenge_sim = ChallengeSimulation()
        challenge_sim.run()
        rospy.spin()
    except rospy.ROSInitException:
        rospy.loginfo('Could not start challenge simulation.')


if __name__ == '__main__':
    main()