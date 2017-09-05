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
import rospkg
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
        # Selects objects to send to the collision checking
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

    def get_score(self,traj_length, vel_change, acc_change, collision_dist, manipulability):
        # Variables
        num_traj = len(traj_length)
        max_length = 4.0
        max_acc_change = 0.007
        max_vel_change = 0.01
        min_collision_distance = 0.01
        max_manip = 0.3
        # Weights
        w_acc = 1.0
        w_vel = 1.0
        w_length = 1.0
        w_collision = 0.1
        w_man = 1
        score = []
        text_line = []

        for n,traj in enumerate(traj_length):
            if collision_dist[n] == -1:  # If collision
                score.append(100)
                text_line.append('& \multicolumn{6}{c|}{Collision Found} \ \n')
            else:
                score.append(w_length * traj / max_length + w_vel * vel_change[n] / max_vel_change \
                             + w_acc * acc_change[n] / max_acc_change \
                             + w_collision * collision_dist[n] / min_collision_distance
                             + w_man *( max_manip - manipulability[n]))
                text_line.append( '& ' + str(round(traj*100,2)) + ' & ' + str(round(vel_change[n],3)) + ' & '
                                  + str(round(acc_change[n],3)) + ' & ' + str(round(collision_dist[n]*100,2)) +
                                  ' & ' + str(round(manipulability[n],3)) + ' & '
                                  + str(round(score[n],3)) + ' \ \n')

        for x in range(5-num_traj):
            text_line.append('& \multicolumn{6}{c|}{Trajectory failed} \ \n')

        pack = rospkg.RosPack()
        dir = pack.get_path('iai_trajectory_generation_boxy') + '/test_scores/scores.tex'
        text_start = '\multirow{5}{1.5cm}{' + self.goal_obj + '}'
        with open(dir, 'a') as f:
            f.write(text_start)
            for line in text_line:
                f.write(line)
            f.write('\hline')

        return score

    def evaluation(self, request):
        # Collision checking
        print 'Waiting for collision service'
        rospy.wait_for_service('collision_evaluation')
        self.collision_service = rospy.ServiceProxy('collision_evaluation', CollisionEvaluation)

        # Variable definition
        manipulability = request.manipulability
        self.goal_obj = request.object_to_grasp
        traj_length  = request.trajectories_length
        acc_change = []
        vel_change = []
        collision_dist = []
        rospy.loginfo('Service TrajectoryEvaluation: Evaluating trajectories')

        # Calculate change in velocity, in acceleration and iter of each trajectory
        for trajectory in request.trajectories:
            vel, acc, l = self.velocity_change(trajectory)
            if vel:
                vel_change.append(vel)
                acc_change.append(self.acceleration_change(acc, l))
            obj_list = self.select_objects()

            try:
                dist = self.collision_service(trajectory, obj_list)
                collision_dist.append(dist.min_collision_distance)
            except rospy.ServiceException, e:
                print "Service CollisionEvaluation call failed: %s" % e

        scores = self.get_score(traj_length, vel_change, acc_change, collision_dist, manipulability)

        print 'scores; ',scores
        if min(scores) == 100:
            rospy.loginfo('Service TrajectoryEvaluation: No trajectory without collision was found')
            return -1
        else:
            selected_traj = scores.index(min(scores))
            rospy.loginfo(
                'Service TrajectoryEvaluation: Selected trajectory: {}, Score: {}'.format(selected_traj, min(scores)))
            return selected_traj


def evaluate_traj():
    rospy.init_node('trajectory_evaluation_server')
    TrajEvaluation()
    rospy.spin()


if __name__ == "__main__":
    evaluate_traj()
