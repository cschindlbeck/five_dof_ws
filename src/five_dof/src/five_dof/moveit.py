"""
This script is for quickly testing path planning with 5 DOF and collision avoidance
"""

import rospy
import copy
import numpy as np
import moveit_commander
from moveit_commander.planning_scene_interface import PlanningSceneInterface

from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf.transformations import quaternion_from_euler


class MoveItInterface:

    def __init__(self, set_goal_orientation: bool = True):
        self.set_goal_orientation = set_goal_orientation
        movegroup = "complete"
        self.group = moveit_commander.MoveGroupCommander(movegroup)
        self.group.set_planner_id("RRTConnect")  # didnt work

    def move_arm(self, posestamped):
        self.group.clear_pose_targets()
        self.group.set_pose_target(posestamped)

        # Set configuration
        # self.group.set_end_effector_link()
        # self.group.set_max_velocity_scaling_factor(1)
        # self.group.set_max_acceleration_scaling_factor(1)
        # self.group.set_start_state_to_current_state()
        # self.group.set_goal_position_tolerance(value=0.4)
        if self.set_goal_orientation:
            self.group.set_goal_orientation_tolerance(np.deg2rad(30))
        # self.group.set_planning_time(40)  # default is 5.0 sec
        # self.group.set_num_planning_attempts(20)

        # self.group.allow_looking(True)
        # self.group.allow_replanning(True)

        (success, _, planning_time, error_code) = self.group.plan()
        if not success:
            rospy.logerr(f"Path planning failed with error code {error_code} in {planning_time} seconds")

        successful_execution = self.group.go(wait=True)
        if not successful_execution:
            rospy.logerr("Execution failed")

        self.group.clear_pose_targets()


class MoveItTest:

    def __init__(self, set_goal_orientation: bool = False):
        self.set_goal_orientation = set_goal_orientation

        new_stamped_pose = PoseStamped()
        new_stamped_pose.header.frame_id = "base_link"
        new_stamped_pose.header.stamp = rospy.Time.now()
        new_stamped_pose.pose = Pose()
        new_stamped_pose.pose.position.x = 0.4
        new_stamped_pose.pose.position.y = -0.1
        new_stamped_pose.pose.position.z = 0.931
        new_stamped_pose.pose.orientation = Quaternion(
            *quaternion_from_euler(0.00000, 0.00000, 0.00000, axes="rxyz"))

        self.pose = new_stamped_pose

    def run(self):
        mif = MoveItInterface(set_goal_orientation=self.set_goal_orientation)
        mif.move_arm(self.pose)
