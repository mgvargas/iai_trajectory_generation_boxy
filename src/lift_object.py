#!/usr/bin/env python
# Copyright (c) 2017 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#
# Author: Minerva Gabriela Vargas Gleason <minervavargasg@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
import tf2_ros
import actionlib
from iai_trajectory_generation_boxy.msg import ProjectedGraspingActionResult
from iai_trajectory_generation_boxy.msg import RequestTrajectoryAction, RequestTrajectoryGoal
from iai_wsg_50_msgs.msg import PositionCmd

class Lifting:
    def __init__(self):
        self.grip_left = 'left_gripper_tool_frame'
        self.grip_right = 'right_gripper_tool_frame'

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.gp_action = actionlib.SimpleActionClient('request_trajectory', RequestTrajectoryAction)
        self.pub_trajectory = rospy.Publisher("projected_grasping_server/result", ProjectedGraspingActionResult, queue_size=3)

    def get_pose(self):
        t = ProjectedGraspingActionResult()
        arm = rospy.get_param('arm')
        if arm == 'left':
            self.gripper_pose = self.grip_left
            self.gripper =  "left_arm_gripper/goal_position"
        else:
            self.gripper_pose = self.grip_right
            self.gripper = "right_arm_gripper/goal_position"

        self.close_gripper()

        try:
            pose = self.tfBuffer.lookup_transform('odom', self.gripper_pose, rospy.Time(0), rospy.Duration(2, 5e8))

        except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.LookupException,
                    tf2_ros.InvalidArgumentException):
            rospy.logerr('No TF found between gripper and object.')
        else:
            pose.transform.translation.z += 0.08

            res, state = self.call_gp_action(pose, arm)
            if state == 'SUCCEEDED':
                t.result.selected_trajectory.trajectory = res.trajectory
                self.pub_trajectory.publish(t)
            else:
                pose.transform.translation.z -= 0.03
                t, state = self.call_gp_action(pose, arm)
                if state == 'SUCCEEDED':
                    t.result.selected_trajectory.trajectory = res.trajectory
                    self.pub_trajectory.publish(t)
                else:
                    rospy.logerr('No trajectory found for lifting position')

    def close_gripper(self):
        pub_gripper = rospy.Publisher(self.gripper, PositionCmd, queue_size=3)
        rospy.sleep(0.5)
        close = PositionCmd()
        close.pos = 30.0
        close.speed = 60.0
        close.force = 30.0
        print "Closing gripper", self.gripper
        pub_gripper.publish(close)
        rospy.sleep(2)

    def call_gp_action(self, pose, arm):
        self.gp_action.wait_for_server()

        goal = RequestTrajectoryGoal(grasping_pose=pose, pre_grasping=pose, arm=arm)
        self.gp_action.send_goal(goal)
        state_string = self.action_state_to_string()

        rospy.loginfo('Sending goal to lift the object.')
        wait_for_result = self.gp_action.wait_for_result(rospy.Duration.from_sec(14))

        if wait_for_result:
            rospy.sleep(0.05)
            state = state_string[self.gp_action.get_state()]
            rospy.loginfo('Action state: {}.'.format(state))
            if state == 'SUCCEEDED':
                rospy.loginfo('Action Result: Trajectory generated.')

        else:
            state = state_string[self.gp_action.get_state()]
            if state == 'ABORTED':
                rospy.loginfo('Action state: {} by action server. \n'.format(state))
            else:
                self.gp_action.cancel_goal()
                self.gp_action.cancel_all_goals()
                rospy.loginfo('Action did not finish before the time out. Cancelling goal.')
                rospy.sleep(0.08)
                state = state_string[self.gp_action.get_state()]
                rospy.loginfo('Action state: {}. \n'.format(state))
        res = self.gp_action.get_result()

        rospy.set_param('should_lift', False)
        return res, state

    @staticmethod
    def action_state_to_string():
        state = {
            0: 'PENDING',
            1: 'ACTIVE',
            2: 'PREEMPTED',
            3: 'SUCCEEDED',
            4: 'ABORTED',
            5: 'REJECTED',
            6: 'PREEMPTING',
            7: 'RECALLING',
            8: 'RECALLED',
            9: 'LOST'}
        return state


def lift():
    command = Lifting()
    #while not rospy.is_shutdown():
    command.get_pose()


if __name__ == '__main__':
    try:
        rospy.init_node('lift_object')
        rospy.Rate(200)
        lift()
    except rospy.ROSInterruptException:
        pass
