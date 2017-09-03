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
import numpy as np
from iai_trajectory_generation_boxy.srv import TrajectoryEvaluation, CollisionEvaluation
from visualization_msgs.msg import MarkerArray


class TrajEvaluation:
    def __init__(self):
        # Variables
        self.goal_obj = 'none'

        rospy.Service('trajectory_evaluation', TrajectoryEvaluation, self.evaluation)
        # Find objects. Subscribe to topic where detected objects are published.
        rospy.Subscriber("visualization_marker_array", MarkerArray, self.marker_callback)

    def marker_callback(self, msg):
        self.all_markers = msg

    def select_objects(self):
        obj_list = MarkerArray()
        self.obj_names = []

        for obj in self.all_markers.markers:
            # If marker is not a grasping pose
            if 'gp' not in obj.ns:
                if obj.ns not in self.obj_names and self.goal_obj != obj.ns:
                    self.obj_names.append(obj.ns)
                    obj_list.markers.append(obj)
        return obj_list

    def velocity_change(self, traj):
        trajectory = traj.trajectory
        joint_num = len(trajectory[0].velocity)
        old_vel = np.zeros(joint_num)
        difference = []
        i = 0
        for x in trajectory:
            vel = np.array(x.velocity)
            difference.append(abs(old_vel - vel))
            old_vel = vel
            i += 1

        acc_sum = np.zeros(joint_num)

        for step in difference:
            for x, joint in enumerate(step):
                acc_sum[x] += joint

        accel_average = [x/i for x in acc_sum]
        # Remove inactive joints
        acc_active = [x for x in accel_average if x]

        return np.mean(acc_active), difference, i

    def acceleration_change(self, acceleration, iterat):
        joint_num = len(acceleration[0])
        jerk = np.zeros(joint_num)
        old_acc = np.zeros(joint_num)

        difference = []
        for acc in acceleration:
            difference.append(abs(old_acc - acc))
            old_acc = np.array(acc)

        for step in difference:
            for x, joint in enumerate(step):
                jerk[x] += joint

        jerk_average = [x/iterat for x in jerk]
        jerk_active = [x for x in jerk_average if x]

        return np.mean(jerk_active)

    def evaluation(self, request):
        # Collision checking
        print 'Waiting for collision service'
        rospy.wait_for_service('collision_evaluation')
        self.collision_service = rospy.ServiceProxy('collision_evaluation', CollisionEvaluation)

        # Variable definition
        num = len(request.trajectories)
        manipulability = request.manipulability
        self.goal_obj = request.object_to_grasp
        print 'length: ', request.trajectories_length
        acc_change = []
        vel_change = []
        iter = []
        collision_dist = []
        grades = [0]*num
        rospy.loginfo('Service TrajectoryEvaluation: Evaluating trajectories')

        # Calculate change in velocity, in acceleration and iter of each trajectory
        for trajectory in request.trajectories:
            vel, acc, l = self.velocity_change(trajectory)
            if vel:
                vel_change.append(vel)
                acc_change.append(self.acceleration_change(acc, l))
                iter.append(l)
            obj_list = self.select_objects()

            try:
                dist = self.collision_service(trajectory, obj_list)
                collision_dist.append(dist.min_collision_distance)
            except rospy.ServiceException, e:
                print "Service CollisionEvaluation call failed: %s" % e

        '''print 'vel_change: ',vel_change
        print 'acc_change: ',acc_change
        print 'len: ',iter'''

        min_vel = vel_change.index(min(vel_change))
        min_acc = acc_change.index(min(acc_change))
        min_length = iter.index(min(iter))
        max_manip = manipulability.index(max(manipulability))
        grades[min_vel] += 1
        grades[min_acc] += 1
        grades[min_length] += 1.5
        grades[max_manip] += 2
        selected_traj = grades.index(max(grades))
        rospy.loginfo('---- Collision Distance: {}'.format(collision_dist))

        rospy.loginfo('Service TrajectoryEvaluation: Selected trajectory: {}'.format(selected_traj))

        return selected_traj


def evaluate_traj():
    rospy.init_node('trajectory_evaluation_server')
    TrajEvaluation()
    rospy.spin()


if __name__ == "__main__":
    evaluate_traj()
