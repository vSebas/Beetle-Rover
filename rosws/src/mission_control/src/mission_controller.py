"""
The mission controller commands the robot movement so that it visits each target cube once.

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

4) How to use an obstacle publisher?
   * http://wiki.ros.org/teb_local_planner/Tutorials/Incorporate%20customized%20Obstacles

5) How to cmd the rover to rotate?
    * http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right

6) How to rotate a quaternion?
    * http://wiki.ros.org/tf2/Tutorials/Quaternions 
"""

import math
import sys
import random
from threading import Thread

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Transform, Twist, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from tf.transformations import quaternion_from_euler, quaternion_multiply

from cube import Cube

class MissionController(object):
    """
    The mission controller will coordinate all relevant
    subcomponents to try to achieve the required tasks
    """

    def __init__(self, n_total_cubes=7, n_target_cubes=4, global_frame_id='world'):
        rospy.on_shutdown(self._shutdown)

        # Global frame id can vary in simulation and real scenarios
        self._global_frame_id = global_frame_id
        # Keeps a record of cubes pending to be discovered
        self._pending_cubes = []
        # Keeps a record of visited target cubes
        self._visited_cubes = []
        # Keeps a record of discovered target cubes pending to be visited
        self._unvisited_cubes = []
        # Keeps a record of discovered obstacle cubes
        self._obstacle_cubes = []
        # Indicates the total amount of cubes the map should contain
        self._n_total_cubes = n_total_cubes
        # Indicates the amount of target cubes the rover needs to visit
        # in order to complete the mission
        self._n_target_cubes = n_target_cubes

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._cmd_vel_publisher = rospy.Publisher('base_footprint/cmd_vel', Twist, queue_size=5)
        self._obstacles_publisher = rospy.Publisher('move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=5)

        self._cube_discovery_timer = None

    def initialization(self):
        random.seed()
        self._pending_cubes = [Cube(i+1) for i in range(self._n_total_cubes)]
        # Shutdown this timer when all cubes are discovered.
        self._cube_discovery_timer = rospy.Timer(rospy.Duration(1.0/10.0), self._handle_cube_discovery_timeout)

    def finalize(self):
        if self._cube_discovery_timer is not None:
            self._cube_discovery_timer.shutdown()
            self._cube_discovery_timer = None
        
    def yaw_rotation_scan(self):
        vel_msg = Twist()

        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        angular_speed = 0.5 # rad/s
        vel_msg.angular.z = angular_speed # CCW rotation in yaw axis

        current_angle = math.radians(0)
        target_angle = math.radians(360)
        t0 = rospy.Time.now().to_sec()

        while current_angle < target_angle:
            self._cmd_vel_publisher.publish(vel_msg)
            t_now = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t_now-t0)
        vel_msg.angular.z = 0
        self._cmd_vel_publisher.publish(vel_msg)

    def target_cubes_pending_visit(self):
        return not self.all_target_cubes_visited()

    def all_target_cubes_visited(self):
        return len(self._visited_cubes) == self._n_target_cubes    

    def visit_next_target_cube(self):
        next_goal = self._nearest_unvisited_cube()
        success = False
        if next_goal is not None:
            success = self._visit_cube(next_goal)
        return success
    
    def explore(self):
        for _ in range(15):
            self._explore_step()

    def _explore_step(self):
        # Get current pose
        rovt = self._get_rover_transformation()
        new_xpos = rovt.transform.translation.x + self._uniform_dist(0.12, 0.20) * (1 if self._bernoulli_trial(0.5) else -1)
        new_ypos = rovt.transform.translation.y + self._uniform_dist(0.12, 0.20) * (1 if self._bernoulli_trial(0.5) else -1)
        # Get current orientation's quaternion
        q = rovt.transform.rotation
        q1 = [q.x, q.y, q.z, q.w]
        # Generate a random orientation angle
        yaw_angle = random.choice([-90, -45, 0, 45, 90, 180])
        q2 = quaternion_from_euler(0, 0, math.radians(yaw_angle))
        # Create rotated quaternion
        q_rot = quaternion_multiply(q1, q2)
        new_orientation = Quaternion(*q_rot)
        # Create new goal
        self._move_base_client.wait_for_server(rospy.Duration(2.0))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self._global_frame_id
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = new_xpos
        goal.target_pose.pose.position.y = new_ypos
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation = new_orientation

        if not self._move(goal):
            rospy.loginfo('Failed to move to random location')

    def _uniform_dist(self, a=0.0, b=1.0):
        sample = a + (b-a)*random.random()
        assert(sample < b)
        assert(sample >= a)
        return sample

    def _bernoulli_trial(self, p):
	    assert(p <= 1.0)
	    return self._uniform_dist(0.0, 1.0) < p

    def _get_rover_transformation(self):
        """
        Returns rover's position and orientation from the 
        base_footprint frame in relation to the world's frame
        """
        return self._get_frame_transform(self._global_frame_id, 'base_footprint')

    def _get_cube_transformation(self, cube):
        """
        Returns the transform from the cube's coordinate frame
        to the world's coordinate frame
        """
        return self._get_frame_transform(self._global_frame_id, cube.frame_id)

    def _publish_obstacles(self):
        obstacle_msg = ObstacleArrayMsg()
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = self._global_frame_id

        for cube in self._obstacle_cubes:
            obstacle = ObstacleMsg()
            obstacle.id = cube.number
            obstacle.polygon.points = cube.forbidden_zone_bounding_box
            obstacle_msg.obstacles.append(obstacle)
        self._obstacles_publisher.publish(obstacle_msg)

    def _euclidean_distance_to_rover(self, cube):
        """
        Returns distance between rover's position and a cube's position
        """
        cube_trans = self._get_cube_transformation(cube)
        rover_trans = self._get_rover_transformation()
        return math.sqrt((cube_trans.transform.translation.x - rover_trans.transform.translation.x)**2 + (cube_trans.transform.translation.y - rover_trans.transform.translation.y)**2)

    def _handle_visited_cube(self, cube):
        self._visited_cubes.append(cube)
        self._unvisited_cubes = filter(lambda c: c.number != cube.number, self._unvisited_cubes)

    def _handle_discovered_cube(self, cube, transform):
        # Save the coordinates where it was discovered in the world frame
        cube.xpos = transform.transform.translation.x
        cube.ypos = transform.transform.translation.y
        cube.orientation = transform.transform.rotation

        rospy.loginfo('Discovered a new cube: {}'.format(cube))

        if cube.is_target:
            rospy.loginfo('Cube is a target')
            self._unvisited_cubes.append(cube)
        else:
            rospy.loginfo('Cube is an obstacle')
            self._obstacle_cubes.append(cube)
            self._publish_obstacles()

    def _handle_cube_discovery_timeout(self, event):
        """
        Listens to the transformations that the aruco_analyzer publishes
        when it detects a cube
        """
        for cube in self._pending_cubes:
            transform = self._get_frame_transform(self._global_frame_id, cube.frame_id, log_error=True)
            if transform:
                self._pending_cubes = filter(lambda c: c.number != cube.number, self._pending_cubes)
                self._handle_discovered_cube(cube, transform)

    def _visit_cube(self, cube):
        """
        Creates a goal to move towards a cube
        and sends that goal to the move base server
        Returns True if the cube was visited
        """
        rospy.loginfo('Attempting to move towards {}'.format(cube))
        self._move_base_client.wait_for_server(rospy.Duration(4.0))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self._global_frame_id
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = cube.approach_zone_xpos
        goal.target_pose.pose.position.y = cube.approach_zone_ypos
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation = cube.orientation

        if self._move(goal):
            self._handle_visited_cube(cube)
            return True
        else:
            rospy.loginfo('Failed to move towards {}'.format(cube))
            return False

    def _nearest_unvisited_cube(self):
        if self._unvisited_cubes:
            return min(self._unvisited_cubes, key=self._euclidean_distance_to_rover)
        return None

    def _mission_finished(self):
        return len(self._visited_cubes) == self._n_target_cubes

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

    def _get_frame_transform(self, target_frame, source_frame, max_retries=1, log_error=True, at_time=0, max_duration=1.0):
        """
        Returns the latest transform from source_frame's coordinate frame
        to the target_frame's coordinate frame
        """
        transform = None
        current_retries = 0
        while transform is None and (current_retries < max_retries):
            try:
                transform = self._tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(at_time), rospy.Duration(max_duration))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                if log_error:
                    rospy.logerr('Error getting frame transform ({} -> {}): {}'.format(source_frame, target_frame, e))
                current_retries += 1
        return transform

    def _shutdown(self):
        """ Taken from source 1) """
        rospy.loginfo('Stopping the robot...')
        # Cancel any active goals
        self._move_base_client.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self._cmd_vel_publisher.publish(Twist())
        rospy.sleep(1)